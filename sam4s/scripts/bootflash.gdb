# Set the Architecture to arm
set architecture arm

# Set bit to boot from flash rather than running bootloader
# You should see it change from the first command to the third here
monitor at91sam4 gpnvm
monitor at91sam4 gpnvm set 1
monitor at91sam4 gpnvm
