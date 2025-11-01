using JSON, Random

function rand_poisson(λ::Float64)
    L = exp(-λ)
    k = 0
    p = 1.0
    while true
        k += 1
        p *= rand()
        if p <= L
            return k - 1
        end
    end
end

function make_tc_poisson_13h(; seed=1234,
    hours=12, lambda_per_hour = 75,
    outpath="case/tc_poisson_13h.json")

    Random.seed!(seed)

    minutes = hours * 60
    # T_end   = minutes + w_max + tau
    T_end   = minutes + 60 #! Add one hour for Don't haave Stranded passengers
    λm  = lambda_per_hour / 60.0

    arr = Dict{String,Int}()
    pid = 1

    for m in 0:(minutes-1)
        for _ in 1:rand_poisson(λm)
            arr[string(pid)] = m
            pid += 1
        end
    end

    cfg = Dict(
        "T" => "0:$(T_end+60)", #! Add one hour for Don't haave Stranded passengers
        "L" => 1,
        "lambda" => lambda_per_hour,
        "arrivals" => Dict("TERMINAL"=>arr),
    )

    mkpath(dirname(outpath))
    open(outpath, "w") do io
        write(io, JSON.json(cfg))
    end
    println("Wrote $(outpath)")
    println("Arr pax = ", length(arr))
    println("T = 0:$(T_end)")
end

# ---------- Parse command-line arguments ----------
#   julia make_tc_poisson_13h.jl --seed=42 --lambda_per_hour=120.0
function parse_and_run()
    opts = Dict(
        "seed" => 1234,
        "hours" => 12,
        "lambda_per_hour" => 75.0,
        "outpath" => "case/tc_poisson_13h.json"
    )

    for arg in ARGS
        if occursin("=", arg)
            key, val = split(arg, "=", limit=2)
            key = replace(key, r"^--" => "")
            if haskey(opts, key)
                if key in ["lambda_per_hour"]
                    opts[key] = parse(Float64, val)
                elseif key in ["outpath"]
                    opts[key] = val
                else
                    opts[key] = parse(Int, val)
                end
            else
                @warn "Unknown parameter $key"
            end
        end
    end

    make_tc_poisson_13h(
        seed=opts["seed"],
        hours=opts["hours"],
        lambda_per_hour=opts["lambda_per_hour"],
        outpath=opts["outpath"]
    )
end

parse_and_run()
