# kalman
Implementation of the Kalman Filter Algorithm in Julia.
---

If you only want the implementation, it is the `KalmanFilter.jl`.
Otherwise still feel free to experiment with the data.

- `00.stochasticMass` shows how the filter puts the pdf Function to the ground thruth.
- `01.sealevel` shows the filter in work, applied to the global mean sea level dataset.
- `02.sinus` applies the filter to a non linear system.
- `03.carposition` shows the usage of the filter in motion filtering. First through a simulation ist data created and the filter is applied to smooth the data if only the accelerometer or the GPS is known. After that, there is a data fusion performed with both of these values and the data is applied to real motion data.

To run the examples  `julia` is needed.
```julia
$ git clone https://github.com/hydroid7/kalman
$ cd kalman
$ julia
               _
   _       _ _(_)_     |  Documentation: https://docs.julialang.org
  (_)     | (_) (_)    |
   _ _   _| |_  __ _   |  Type "?" for help, "]?" for Pkg help.
  | | | | | | |/ _` |  |
  | | |_| | | | (_| |  |  Version 1.0.3 (2018-12-18)
 _/ |\__'_|_|_|\__'_|  |  Official https://julialang.org/ release
|__/                   |
julia> using IJulia
juila> notebook(dir = pwd())
```
You will find folders with number prefix. Open them and run the exercises.
