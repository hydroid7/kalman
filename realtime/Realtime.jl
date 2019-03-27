include("../KalmanFilter.jl")

using Sockets, JSON, .KalmanFilter, LinearAlgebra

"""
    @loop expression

Repeats the `expression` forever
"""
macro loop(ex)
    quote
        while true
            $(esc(ex))
        end
    end
end

struct Vector3
    x::Number
    y::Number
    z::Number
end
Vector3(dict::Dict) = Vector3(dict["x"], dict["y"], dict["z"])

function getOrDefault(array, default)
    if length(array) == 1
        array[1]
    else
        default
    end
end

function calculateKalman(Δ_t, prev_model::Kalman, acc::Vector3)
    A = [1 0 Δ_t 0 0.5*Δ_t^2 0;
         0 1 0 Δ_t 0 0.5*Δ_t^2;
         0 0 1 0 Δ_t 0;
         0 0 0 1 0 Δ_t;
         0 0 0 0 1 0;
         0 0 0 0 0 1]

    G = [0 0 0 0 1 0;
         0 0 0 0 0 1]

    R = [0.1 0;
         0 0.1]

    Q = I * 0.2

    if (abs(acc.x) + abs(acc.y) + abs(acc.z)) < .3
        return (model = model, position = Vector3(f[6], f[5], 0))
    end

    newModel = Kalman(A, Q, G, R, prev_model.x̂, prev_model.Σ)
    (model, f, p, g) = next(newModel, [acc.x; acc.y])
    (model = model, position = Vector3(f[6], f[5], 0))
end

function takeInput!(to::Channel)
    @async begin
        udpsock = UDPSocket()
        bind(udpsock, ip"192.168.178.25", 5555)
        @loop begin
            put!(to, String(recv(udpsock)))
        end
    end
end

function parseInput!(from::Channel, to::Channel)
    prev = (timestamp = 0.0, accelerometer = Vector3(0, 0, 0), gyro = Vector3(0, 0, 0), magneto = Vector3(0, 0, 0))
    @async @loop begin
        data = take!(from)
        data = strip.(split(data, ","))
        timestamp = time()
        if data[2] == "3"
            accelerometer = Vector3(parse(Float64, data[3]), parse(Float64, data[4]), parse(Float64, data[5]))
        end
        result = (timestamp = timestamp, accelerometer = accelerometer)
        put!(to, (delta = timestamp - prev.timestamp, accelerometer = result.accelerometer))
        prev = result
    end
end

function calculateInput!(from::Channel, to::Channel)
    x_0 = zeros(6, 1)
    Σ_0 = Matrix{Float64}(I, 6, 6) .* 1000
    model = Kalman(I, I, I, I, x_0, Σ_0)
    @async @loop begin
        task = take!(from)
        result = calculateKalman(task.delta, model, task.accelerometer)
        model = result.model
        pos = result.position
        out = Dict(
            "timestamp" => time(),
            "position" => Dict("x" => pos.x, "y" => pos.y, "z" => pos.z)
        )
        put!(to, out)
    end
end

function sendToRenderer!(from::Channel, bindPort::Int64)
    @async begin
        outputSocket = UDPSocket()
        @loop begin
            send(outputSocket, ip"127.0.0.1", bindPort, json(take!(from)))
        end
    end
end

inputChanel = Channel(16) # Receives the input
parsedChannel = Channel(16) # Receives parsed data
outputChannel = Channel(16) # It's content is pushed to the rust server.
calculatingChannel = Channel(16)

takeInput!(inputChanel)
parseInput!(inputChanel, calculatingChannel)
calculateInput!(calculatingChannel, outputChannel)
sendToRenderer!(outputChannel, 2000)

@loop begin
    sleep(10)
    @show "Running"
end
