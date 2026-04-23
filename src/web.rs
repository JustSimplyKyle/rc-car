use core::str::FromStr;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use picoserve::{
    extract::Form,
    response::{File, IntoResponse},
    routing::{self, post},
    AppBuilder,
};

use defmt::{info, warn};
use esp_radio::wifi::{ModeConfig, WifiDevice};

use embassy_net::{Runner, StackResources, StaticConfigV4};

use esp_radio::wifi::AccessPointConfig;

use esp_radio::wifi::WifiEvent;

use esp_radio::wifi::WifiApState;

use esp_radio::wifi::WifiController;

use embassy_executor::Spawner;

use embassy_time::Duration;

use embassy_time::Timer;

use embassy_net::Stack;
use picoserve::AppRouter;
use serde::Deserialize;
use static_cell::make_static;

use crate::mk_static;

pub static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, CommandType, { WEB_POOL_SIZE * 2 }> =
    Channel::new();

#[derive(Deserialize, Debug)]
#[serde(rename_all = "snake_case")]
#[serde(tag = "cmd", content = "status")]
pub enum CommandType {
    GoFront(Status),
    GoBack(Status),
    TurnLeft(Status),
    TurnRight(Status),
    TurnFront(Status),
    PullUp(Status),
    PullDown(Status),
    ArmUp(Status),
    ArmDown(Status),
    BlinkRate(u64),
    FrequencyKilohertz(u32),
    PwmPercentage(u8),
    ServoDelay(u64),
    Heartbeat,
}

#[derive(Deserialize, Debug)]
#[serde(rename_all = "snake_case")]
pub enum Status {
    Pressed,
    Released,
    BlinkOnce,
}

pub struct Application;

macro_rules! static_routes {
    ($base:literal, $(
        $route:literal
    ),* $(,)?) => {{
        picoserve::Router::new()
        $(
            .route(
                {
                    if $route == "index.html" {
                        "/"
                    } else {
                        concat!("/", $route)
                    }
                },
                routing::get_service({
                    let content_type = if $route.ends_with(".js") {
                        "application/javascript"
                    } else if $route.ends_with(".css") {
                        "text/css"
                    } else if $route.ends_with(".wasm") {
                        "application/wasm"
                    } else if $route.ends_with(".html") {
                        "text/html"
                    } else {
                        "application/octet-stream"
                    };
                    let bd = include_bytes!(concat!($base, "/", $route));
                    File::with_content_type(content_type, bd)
                }),
            )
        )*
    }};
}

async fn handle_command(Form(form): Form<CommandType>) -> impl IntoResponse {
    match COMMAND_CHANNEL.try_send(form) {
        Ok(_) => {
            // info!("Free heap: {} bytes", HEAP.free());
            "Command Sent"
        }
        Err(_) => {
            warn!("Command Queue Full!");
            "Busy"
        }
    }
}

const STATIC_IP: &str = "192.168.2.1/24";
const GATEWAY_IP: &str = "192.168.2.1";

async fn wait_for_connection(stack: Stack<'_>) {
    info!("Waiting for link to be up");
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}

pub async fn start_wifi(
    radio_init: &'static esp_radio::Controller<'static>,
    wifi: esp_hal::peripherals::WIFI<'static>,
    rng: esp_hal::rng::Rng,
    spawner: &Spawner,
) -> Stack<'static> {
    let (wifi_controller, interfaces) = esp_radio::wifi::new(radio_init, wifi, Default::default())
        .expect("Failed to initialize Wi-Fi controller");

    let wifi_interface = interfaces.ap;
    let net_seed = rng.random() as u64 | ((rng.random() as u64) << 32);
    let Ok(ip_addr) = embassy_net::Ipv4Cidr::from_str(STATIC_IP) else {
        info!("Invalid STATIC_IP");
        loop {}
    };

    let Ok(gateway) = core::net::Ipv4Addr::from_str(GATEWAY_IP) else {
        info!("Invalid GATEWAY_IP");
        loop {}
    };

    let net_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: ip_addr,
        gateway: Some(gateway),
        dns_servers: Default::default(),
    });
    // Init network stack
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        net_config,
        make_static!(StackResources::<{ WEB_POOL_SIZE + 3 }>::new()),
        net_seed,
    );

    spawner.spawn(connection(wifi_controller)).ok();
    spawner.spawn(net_task(runner)).ok();

    wait_for_connection(stack).await;

    stack
}

pub const WEB_POOL_SIZE: usize = 4;

// #[embassy_executor::task(pool_size = WEB_POOL_SIZE)]
// pub async fn web_task(
//     id: usize,
//     stack: Stack<'static>,
//     app: &'static AppRouter<Application>,
//     config: &'static picoserve::Config,
// ) {
//     let mut tcp_rx = [0u8; 1024];
//     let mut tcp_tx = [0u8; 1024];
//     let mut http_buf = [0u8; 2048];
//     let port = 80;

//     info!("Web server listening on port {}", port);

//     picoserve::Server::new(&app, &config, &mut http_buf)
//         .listen_and_serve(id, stack, port, &mut tcp_rx, &mut tcp_tx)
//         .await;
// }

// pub async fn start_web_server(spawner: Spawner, stack: embassy_net::Stack<'static>) {
//     info!("Starting web server with {} tasks...", WEB_POOL_SIZE);

//     let app = make_static!(Application.build_app());

//     let config = make_static!(picoserve::Config::new(picoserve::Timeouts {
//         start_read_request: Duration::from_secs(5).into(),
//         persistent_start_read_request: Duration::from_secs(1).into(),
//         read_request: Duration::from_secs(1).into(),
//         write: Duration::from_secs(1).into(),
//     })
//     .keep_connection_alive());

//     for id in 0..WEB_POOL_SIZE {
//         spawner.must_spawn(web_task(id, stack, app, config));
//     }
// }

#[embassy_executor::task]
pub async fn connection(mut controller: WifiController<'static>) {
    info!("start connection task");
    // info!("Device capabilities: {:?}", &controller.capabilities());
    loop {
        match esp_radio::wifi::ap_state() {
            WifiApState::Started => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::ApStop).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }

        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ModeConfig::AccessPoint(
                AccessPointConfig::default()
                    .with_ssid("esp32".into())
                    .with_password("password".into())
                    .with_auth_method(esp_radio::wifi::AuthMethod::Wpa2Personal),
            );
            controller.set_config(&client_config).unwrap();
            info!("Starting wifi");
            controller.start_async().await.unwrap();
            info!("Wifi started!");
        }
    }
}

#[embassy_executor::task]
pub async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}
