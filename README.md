# SRTL (Simple Real-Time Layer)

## Project Overview

SRTL is a lightweight, modular real-time operating system (RTOS) layer designed for embedded systems, particularly targeting the ESP32 platform. It provides an abstraction for task management, shared resource access, and inter-task communication, leveraging FreeRTOS capabilities while maintaining a simple interface for developers. The core library, **CORE**, allows users to easily create modules, manage shared resources, and handle synchronization via semaphores and notifications.

SRTL aims to simplify the development of concurrent systems by allowing multiple tasks to interact safely with shared resources, with a particular focus on minimizing memory usage and maximizing performance.

---

## Features

- **CORE**: Provides basic task management, resource sharing, and synchronization. It allows easy creation of tasks and communication through shared resources.
- **IHM**: Easily create secure IN/OUT for interracting with the final users
- **Modular System**: The system is designed to be highly modular, enabling the development of complex embedded systems with minimal overhead and no interraction with other library.
- **Inter-Task Communication**: Tasks can notify each other and synchronize through semaphores and notifications.
- **Task Prioritization**: Dynamically adjust task priorities based on resource contention, ensuring fair execution.
- **Cross-Platform**: Currently optimized for the ESP32 platform but can be extended to other microcontrollers that support FreeRTOS speacially 32bits platform.

---

## Getting Started

### Requirements

- **PlatformIO**: The project is compiled using PlatformIO, a popular IDE for embedded systems.
- **ESP32-Wroom 32D**: Tested on the ESP32 Wroom 32D, but should work with other ESP32 boards.

### Installation

1. Clone the repository or download the source files.
2. Open PlatformIO IDE and create a new project.
3. Add the source files into the `src` folder of your project.
4. Include the necessary dependencies (FreeRTOS, etc.) in the `platformio.ini` configuration file.
5. Build and upload the code to your ESP32 device.

### Example Usage

A minimal example is provided in `main.cpp`, where multiple tasks (producers and consumers) interact with shared resources:

```cpp
TODO
```

In this example:
TODO


### Build and Upload
Once the project is set up, use PlatformIO to build and upload the code to your ESP32 device. The tasks will run concurrently, with the producer modifying shared resources and the consumer processing them.

---

## Roadmap

The current implementation focuses on the **CORE** and **IHM** module, which allows the creation of tasks and communication through shared resources with synchronization, and create simple interface called Monitor and Controller to make app interactive. The minimal version of the **CORE** module is functional, and various implementations can be built using it. I need to further test **IHM**.

### Key milestones:
1. **CORE Module**: 
   - ✅ Full support for creating tasks and managing shared resources with synchronization.
   - ✅ Current version is fully functional, with task creation, resource sharing, and notification systems in place. In theory, this feature can be used to set up virtually any concurrency system possible 
   - 📝 Refine semaphore and notification mechanisms for more intuitive use.
   - ⌛  Currently Support 16 modules and 16 shared resources, want to up to 32 each

2. **IHM (Special Modules)**: 
   - ⌛  Introduce user interface modules that will allow interaction with physical devices such as buttons and screens. FSM for behavior and ISR for triggering. Controller manage two type of input, digital (with ISR) and analog (with timer),Monitor is just a special case of Module.

3. **SYNC (Storage and Communication)**: 
   - ⌛ No work has been done yet on the synchronization of storage or communication. Future plans include developing this part to allow simplify communication of shared resource via BLE and some external network protocols (such as MQTT), either via local storage.

4. **Advanced Features**:
   - ⌛ Many templates of concurrency pattern and use cases
   - ⌛ Future versions will introduce advanced features such as multi-resource transactions, priority management, and enhanced UI capabilities.

---

## Contributing

Contributions to the project are welcome! Please fork the repository, make your changes, and submit a pull request. We encourage the community to help improve the system, lighten and accelerate it.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

--- 
