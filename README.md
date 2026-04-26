# Project Description
## tl;dr
a esp-32 powered rc-car, with a step-motor serving as its primary transmission force, using belt wheels for synchronzied left-right wheel, utilizing a mg996r for turning curcuits, displaying all infos on a 128x64 oled display, finally using 3d-printed parts for the car assembly

the pcb is designed using KiCad, while we use Rhino for 3d modeling software.
## why
our school hosted a simple car competition, a really simple one where there's just TT-motors wired to a wireless controller.
my friend and i had so much issues with that car, that we decided we want to make a new-and-improved:tm: version of it!
## wiring diagarm
## screenshots
| ![image](https://stasis.hackclub-assets.com/images/1777115996867-j2d8m9.png) | ![image](https://stasis.hackclub-assets.com/images/1777116036268-phmaao.png) |
| :---: | :---: |
| *(pcb layout)* | *(3d view)* |

| Image | Description |
| :---: | :--- |
| ![image](https://stasis.hackclub-assets.com/images/1777116012598-y53cd0.png) | *(schematic)* |

# Project Structure
the root of the project contains the setup of the firmware, written in esp32-rust.
`src/bin` - the actual production code for the esp32
`src/motor` - all the motor control codes are extraplated in here, with mostly complete compile time checking so that timer usage aren't checked at runtime
`src/ps2.rs & src/ps2_controller_task.rs` - the controller reception api
`setup` - the pcb & schemcatic(kicad)
`3d_models` - the 3d models
