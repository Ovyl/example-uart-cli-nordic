FROM nicksinas/gnu-gcc-arm-none-eabi:latest

COPY . .

CMD ["/bin/bash"]