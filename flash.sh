cargo clean
cargo objcopy --release -- -O binary opilio.bin
fwcrypt -e -i opilio.bin -o opilio-en.bin
dfu-util -a 0 -D opilio-en.bin -d 0483:df11
