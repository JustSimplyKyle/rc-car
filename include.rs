impl AppBuilder for Application {
type PathRouter = impl routing::PathRouter;
fn build_app(self) -> picoserve::Router<Self::PathRouter> {
static_routes!(
    "/home/kyle/coding/controller-ui/target/dx/controller-ui/release/web/public",
    "index.html",
    "assets/tailwind-dxh362ac34bad4fab.css",
    "assets/controller-ui-dxh45e347b8fce6964.js",
    "assets/controller-ui_bg-dxh789eddc513b06e99.wasm"
)
.route("/controller", post(handle_command))
}}
