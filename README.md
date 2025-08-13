# Raman Spectroscopy CCD Data Acquisition (STM32 HAL)

This project implements a complete firmware solution for acquiring Raman spectroscopy data using the **TCD1304 Linear CCD Sensor** and an **STM32F401 microcontroller** with **HAL drivers**. It enables high-resolution spectral measurement, real-time USB communication, and scientific data visualization.

## 📜 Features

- **CCD Sensor Integration** – Controls the TCD1304 with precise clock, integration, and readout timing
- **HAL-based Firmware** – Uses STM32 HAL for portability and maintainability  
- **USB CDC Data Transfer** – Streams spectral data directly to a PC
- **CSV Export** – Enables saving captured spectra for post-processing
- **Raman Shift Plotting** – Generates intensity vs. Raman shift graphs
- **Low Noise Operation** – Optimized signal acquisition for spectroscopy
- **Real-time Data Streaming** – Continuous spectrum acquisition and transmission
- **Configurable Integration Time** – Adjustable exposure settings for different samples

## 🛠 Hardware Requirements

- **Microcontroller**: STM32F401CCU6 (or similar Cortex-M4 MCU)
- **Sensor**: TCD1304 Linear CCD (3648 pixels)
- **Interface**: USB (CDC Virtual COM Port)
- **Power Supply**: 3.3V logic, 5V sensor supply
- **Clock**: External crystal oscillator (8MHz recommended)
- **Optional**: Optical setup for Raman spectroscopy (laser, filters, spectrometer)

### Pin Configuration

| STM32 Pin | TCD1304 Pin | Function |
|-----------|-------------|----------|
| PA0       | SH          | Sample & Hold |
| PA1       | ICG         | Integration Clear Gate |
| PA2       | φM          | Master Clock |
| PA3       | OS          | Analog Output |
| GND       | SUB         | Substrate |
| +5V       | VDD         | Power Supply |

## 📂 Project Structure

```
├── Core/
│   ├── Inc/
│   │   ├── main.h              # Main application header
│   │   ├── stm32f4xx_hal_conf.h   # HAL configuration
│   │   ├── stm32f4xx_it.h      # Interrupt handlers
│   │   ├── usb_device.h        # USB device configuration
│   │   └── usbd_cdc_if.h       # USB CDC interface
│   └── Src/
│       ├── main.c              # Main firmware logic
│       ├── stm32f4xx_hal_msp.c # HAL MSP initialization
│       ├── stm32f4xx_it.c      # Interrupt service routines
│       ├── usb_device.c        # USB device implementation
│       └── usbd_cdc_if.c       # USB CDC interface implementation
├── Drivers/
│   ├── STM32F4xx_HAL_Driver/   # STM32 HAL drivers
│   └── CMSIS/                  # ARM CMSIS headers
├── Middlewares/
│   └── ST/STM32_USB_Device_Library/  # USB middleware
├── rmn.pdf                     # Raman system documentation
├── rmn.txt                     # CCD acquisition parameters
├── RamanSpectroscopy.ioc       # STM32CubeMX project file
└── README.md                   # This file
```

## 🚀 Getting Started

### 1️⃣ Prerequisites

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) installed
- USB cable for programming/debugging
- TCD1304 sensor properly connected to STM32F4 GPIOs
- Python 3.x with pandas and matplotlib for data visualization

### 2️⃣ Hardware Setup

1. Connect the TCD1304 sensor according to the pin configuration table
2. Ensure proper power supply (3.3V for logic, 5V for sensor)
3. Connect USB cable between STM32 and PC
4. Verify all connections are secure

### 3️⃣ Build & Flash

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/raman-spectroscopy-stm32.git
   cd raman-spectroscopy-stm32
   ```

2. Open the `RamanSpectroscopy.ioc` file in STM32CubeIDE

3. Generate code and build the project:
   - Click **Project → Build Project**
   - Resolve any compilation errors

4. Flash to your STM32 board:
   - Connect ST-Link debugger
   - Click **Run → Run As → STM32 Cortex-M C/C++ Application**

### 4️⃣ Data Acquisition

1. Connect the board to your PC via USB
2. The device will enumerate as a Virtual COM Port
3. Open a serial terminal at **115200 baud, 8N1**
4. Start data acquisition by sending commands or use the Python script below

## 📊 Data Processing & Visualization

### Python Data Acquisition Script

```python
import serial
import csv
import time
import matplotlib.pyplot as plt
import numpy as np

def acquire_spectrum(port='COM3', baudrate=115200, samples=3648):
    """Acquire spectrum data from STM32 via USB CDC"""
    ser = serial.Serial(port, baudrate, timeout=1)
    
    # Send acquisition command
    ser.write(b'START\n')
    
    spectrum_data = []
    for i in range(samples):
        line = ser.readline().decode('utf-8').strip()
        if line:
            try:
                intensity = int(line)
                spectrum_data.append(intensity)
            except ValueError:
                continue
    
    ser.close()
    return spectrum_data

def save_spectrum_csv(spectrum, filename='spectrum.csv'):
    """Save spectrum with Raman shift calculation"""
    # TCD1304 specifications
    pixel_size = 8.0e-6  # 8 μm pixel pitch
    focal_length = 0.1   # 100mm focal length (adjust for your setup)
    grating_density = 1200  # lines/mm (adjust for your grating)
    laser_wavelength = 785  # nm (adjust for your laser)
    
    # Calculate Raman shifts
    pixels = np.arange(len(spectrum))
    angles = np.arctan((pixels - len(spectrum)/2) * pixel_size / focal_length)
    wavelengths = laser_wavelength + (angles * 1e7 / grating_density)
    raman_shifts = (1/laser_wavelength - 1/wavelengths) * 1e7
    
    # Save to CSV
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Pixel', 'Raman_Shift', 'Intensity'])
        for i, (shift, intensity) in enumerate(zip(raman_shifts, spectrum)):
            writer.writerow([i, shift, intensity])
    
    print(f"Spectrum saved to {filename}")
    return raman_shifts

def plot_spectrum(filename='spectrum.csv'):
    """Plot Raman spectrum from CSV file"""
    import pandas as pd
    
    # Load spectrum data
    data = pd.read_csv(filename)
    
    # Create the plot
    plt.figure(figsize=(12, 8))
    plt.plot(data['Raman_Shift'], data['Intensity'], linewidth=1.5)
    plt.xlabel('Raman Shift (cm⁻¹)', fontsize=12)
    plt.ylabel('Intensity (counts)', fontsize=12)
    plt.title('Raman Spectrum', fontsize=14, fontweight='bold')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # Add some styling
    plt.gca().spines['top'].set_visible(False)
    plt.gca().spines['right'].set_visible(False)
    
    plt.show()

# Example usage
if __name__ == "__main__":
    print("Acquiring Raman spectrum...")
    spectrum = acquire_spectrum(port='COM3')  # Adjust COM port
    
    if spectrum:
        print(f"Acquired {len(spectrum)} data points")
        raman_shifts = save_spectrum_csv(spectrum)
        plot_spectrum()
    else:
        print("No data acquired. Check connections and COM port.")
```

### MATLAB Alternative

```matlab
% MATLAB script for Raman spectrum acquisition and analysis
function raman_analysis()
    % Serial port configuration
    s = serialport('COM3', 115200);  % Adjust COM port
    
    % Acquire data
    fprintf('Acquiring spectrum...\n');
    write(s, "START", "string");
    
    spectrum = [];
    for i = 1:3648
        data = readline(s);
        if ~isempty(data)
            spectrum(end+1) = str2double(data);
        end
    end
    
    delete(s);
    
    % Calculate Raman shifts (adjust parameters for your setup)
    pixel_size = 8e-6;      % TCD1304 pixel pitch (m)
    focal_length = 0.1;     % Spectrometer focal length (m) 
    grating = 1200;         % Grating density (lines/mm)
    laser_wl = 785;         % Laser wavelength (nm)
    
    pixels = 1:length(spectrum);
    angles = atan((pixels - length(spectrum)/2) * pixel_size / focal_length);
    wavelengths = laser_wl + (angles * 1e7 / grating);
    raman_shifts = (1/laser_wl - 1./wavelengths) * 1e7;
    
    % Plot spectrum
    figure;
    plot(raman_shifts, spectrum, 'LineWidth', 1.5);
    xlabel('Raman Shift (cm^{-1})');
    ylabel('Intensity (counts)');
    title('Raman Spectrum');
    grid on;
    
    % Save data
    csvwrite('spectrum.csv', [pixels' raman_shifts' spectrum']);
    fprintf('Spectrum saved to spectrum.csv\n');
end
```

## ⚙️ Configuration

### CCD Timing Parameters

The TCD1304 requires precise timing control. Key parameters in `rmn.txt`:

```
Integration_Time = 100ms    # Exposure time
Master_Clock = 2MHz        # φM frequency  
Readout_Rate = 500kHz      # Pixel readout frequency
Sample_Hold_Width = 2μs    # SH pulse width
Integration_Clear = 1μs    # ICG pulse width
```

### Optical Setup Calibration

For accurate Raman shift calculation, calibrate your optical setup:

1. **Measure pixel-to-wavelength mapping** using known calibration lines
2. **Determine dispersion** (nm/pixel or cm⁻¹/pixel)
3. **Update calibration constants** in the Python/MATLAB scripts

## 🔧 Troubleshooting

### Common Issues

| Problem | Solution |
|---------|----------|
| No USB enumeration | Check USB cable, verify CDC driver installation |
| Noisy spectrum | Reduce integration time, check power supply stability |
| Missing data points | Increase serial timeout, check baud rate |
| Incorrect Raman shifts | Calibrate optical setup, verify laser wavelength |

### Debug Commands

Send these commands via serial terminal:

- `START` - Begin spectrum acquisition
- `STOP` - Stop acquisition
- `STATUS` - Check system status  
- `RESET` - Reset CCD sensor
- `CALIB` - Enter calibration mode

## 📈 Performance Specifications

- **Spectral Range**: 200-4000 cm⁻¹ (typical, depends on optical setup)
- **Resolution**: ~1 cm⁻¹ (depends on grating and focal length)
- **Integration Time**: 1ms - 10s (configurable)
- **Readout Speed**: Up to 3648 pixels in ~7ms
- **Dynamic Range**: 16-bit (65536 levels)
- **USB Transfer Rate**: ~1MB/s

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

```
MIT License

Copyright (c) 2025 Nikhil C P

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## 👤 Author

**Nikhil C P**
- Specializing in AI, Robotics, and Embedded Systems
- Passionate about scientific instrumentation and spectroscopy


## 🙏 Acknowledgments

- STMicroelectronics for the excellent STM32 HAL library
- Toshiba for the TCD1304 CCD sensor documentation
- Open-source community for Python scientific libraries
- Contributors who helped improve this project

## 📚 References

1. TCD1304 Linear CCD Datasheet - Toshiba
2. STM32F401 Reference Manual - STMicroelectronics  
3. "Introduction to Raman Spectroscopy" - Spectroscopy Online
4. USB CDC Class Specification - USB Implementers Forum
5. HAL Driver User Manual - STMicroelectronics

---

⭐ **Star this repository if you find it helpful!** ⭐
