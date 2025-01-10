# STM32 Bare-Metal Driver Implementation
A bare-metal UART (Universal Asynchronous Receiver-Transmitter) driver implementation for STM32 Nucleo boards, built from scratch. This project demonstrates low-level driver development for embedded systems, focusing on clean architecture and efficient communication protocols.
## 🚀 Features

Bare-metal UART driver implementation without HAL dependencies
Configurable baud rate settings
Support for different data frame formats (8N1, 8E1, etc.)
Interrupt-driven and polling-based communication modes
Error handling for common UART issues (framing, noise, overrun)
Comprehensive documentation and example usage

## 📋 Prerequisites

STM32 Nucleo Development Board
ARM GCC Toolchain
STM32CubeIDE (optional)
USB-TTL converter for testing (optional)
Basic understanding of UART protocol and ARM Cortex-M architecture

## 🛠️ Hardware Setup

Required Connections
STM32 Nucleo Board    USB-TTL Converter
UART_TX (PA.9)   -->  RX
UART_RX (PA.10)  -->  TX
GND              -->  GND
## 💻 Building and Flashing

### Clone the repository:
bashCopygit clone https://github.com/yourusername/stm32-uart-driver.git
cd stm32-uart-driver

Build the project:
bashCopymake

Flash to your Nucleo board:
bashCopymake flash


## 📚 Project Structure

    .
    stm32-uart-driver/
    ├── src/
    │   ├── uart_driver.c        # UART driver implementation
    │   ├── uart_driver.h        # UART driver header
    │   └── main.c              # Example usage
    ├── inc/
    │   └── stm32_reg.h         # STM32 register definitions
    ├── examples/
    │   ├── echo.c              # Echo example
    │   └── string_tx.c         # String transmission example
    ├── docs/
    │   ├── uart_config.md      # Configuration guide
    │   └── api.md             # API documentation
    ├── Makefile               # Build configuration
    ├── README.md             # Project documentation
    └── LICENSE              # MIT License
> Use short lowercase names at least for the top-level files and indent properly with 2 tabs. 

    
    
## 🔧 Usage Example
#include "uart_driver.h"

int main(void) {
    // Initialize UART with 115200 baud rate
    UART_Config config = {
        .baudrate = 115200,
        .wordLength = UART_WORDLENGTH_8B,
        .stopBits = UART_STOPBITS_1,
        .parity = UART_PARITY_NONE
    };
    
    UART_Init(&config);
    
    // Send string
    UART_SendString("Hello, World!\r\n");
    
    while(1) {
        // Echo received character
        if(UART_IsRxReady()) {
            char c = UART_ReceiveChar();
            UART_SendChar(c);
        }
    }
}
## 📖 Documentation

+ STM32F4 Nucleo Guide
+ API Documentation
+ STM32 Reference Manual

## ⚡ Performance

Supports baud rates up to 921600
Minimal CPU overhead in interrupt mode
~1ms latency for character echo in polling mode
Buffer sizes configurable for different memory constraints

🤝 Contributing

Fork the repository
Create your feature branch (git checkout -b feature/AmazingFeature)
Commit your changes (git commit -m 'Add some AmazingFeature')
Push to the branch (git push origin feature/AmazingFeature)
Open a Pull Request

📝 License
This project is licensed under the MIT License - see the LICENSE file for details.
✨ Acknowledgments

STMicroelectronics for the STM32 Reference Manual
ARM for the Cortex-M architecture documentation
The embedded systems community for their invaluable resources

📞 Contact
Your Name - @yourusername
Project Link: https://github.com/yourusername/stm32-uart-driver
