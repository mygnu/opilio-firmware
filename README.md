# opilio-firmware

Firmware for STM32F103C8 based custom board for PC water-cooling project.


Counterparts: 
- Linux TUI app can be found [here](https://github.com/mygnu/opilio)
- PCB Files [here](https://github.com/mygnu/opilio-pcb)


## Flashing an opilio board
- install rust from [rustup](https://rustup.rs)
- `cargo install probe-run cargo-flash flip-link`
- `git clone https://github.com/mygnu/opilio`
- `git clone https://github.com/mygnu/opilio-firmware`
- `cd opilio-firmware && cargo flash --chip STM32F103C8 --release`
