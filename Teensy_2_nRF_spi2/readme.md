We used SPI3 which has extranal arudino pins for comm b/w teensy and nrf, this worked, but theproblemis whenit is used togeter with dect then the dect's modem uses spi3 by default, so there is a bus conflict, 
the solution is to change the teensy comm to spi2.
