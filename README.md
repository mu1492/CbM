# SpectralViewer
Condition based Monitoring (CbM) for mechanical vibrations. 
Supported accelerometers are Analog Devices [ADXL355](https://www.analog.com/en/products/adxl355.html) and [ADXL357](https://www.analog.com/en/products/adxl357.html).

Types of available plots for each axis:
- transient
- FFT (Fast Fourier Transform)
- periodogram (Power Spectral Density)
- SRS (Shock Response Spectrum)
- cepstrum

**Building SpectralViewer**
> If not on a NVIDIA platform, set OFF the USE_CUDA parameter in the Project config section from CMakeLists.txt.  Depending on this setting, the application will be called either **SpectralViewerCuda** or **SpectralViewer**.

Build steps:

    cd SpectralViewer
    mkdir build
    cd build
    cmake ..
    make
