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
    hours=12, lambda_per_hour_cei=75.0, lambda_per_hour_t2=40.0,
    tau=40, c_max=20, w_max=60, buses=7, outpath="case/tc_poisson_13h.json")

    Random.seed!(seed)

    minutes = hours * 60
    # T_end   = minutes + w_max + tau
    T_end   = minutes + 60 #! Add one hour for Don't haave Stranded passengers
    λm_cei  = lambda_per_hour_cei / 60.0
    λm_t2   = lambda_per_hour_t2  / 60.0

    B = collect(1:buses)
    initpos = [ Dict("bus"=>b, "terminal"=>(b <= buses÷2 ? "CEI" : "T2")) for b in B ]

    arr_cei = Dict{String,Int}()
    arr_t2  = Dict{String,Int}()
    pid_cei = 1
    pid_t2  = 1

    for m in 0:(minutes-1)
        for _ in 1:rand_poisson(λm_cei)
            arr_cei[string(pid_cei)] = m
            pid_cei += 1
        end
        for _ in 1:rand_poisson(λm_t2)
            arr_t2[string(pid_t2)] = m
            pid_t2 += 1
        end
    end

    cfg = Dict(
        "T" => "0:$(T_end+60)", #! Add one hour for Don't haave Stranded passengers
        "B" => B,
        "L" => 1,
        "tau" => tau,
        "c_max" => c_max,
        "w_max" => w_max,
        "arrivals" => Dict("CEI"=>arr_cei, "T2"=>arr_t2),
        "initial_positions" => initpos
    )

    mkpath(dirname(outpath))
    open(outpath, "w") do io
        write(io, JSON.json(cfg))
    end
    println("Wrote $(outpath)")
    println("CEI pax = ", length(arr_cei), "  |  T2 pax = ", length(arr_t2))
    println("T = 0:$(T_end)  tau=$(tau)  c_max=$(c_max)  w_max=$(w_max)  buses=$(buses)")
end

# ---------- Parse command-line arguments ----------
#   julia make_tc_poisson_13h.jl --seed=42 --lambda_per_hour_cei=120.0 --lambda_per_hour_t2=80.0 --buses=10
function parse_and_run()
    opts = Dict(
        "seed" => 1234,
        "hours" => 12,
        "lambda_per_hour_cei" => 75.0,
        "lambda_per_hour_t2"  => 40.0,
        "tau" => 40,
        "c_max" => 20,
        "w_max" => 60,
        "buses" => 7,
        "outpath" => "case/tc_poisson_13h.json"
    )

    for arg in ARGS
        if occursin("=", arg)
            key, val = split(arg, "=", limit=2)
            key = replace(key, r"^--" => "")
            if haskey(opts, key)
                if key in ["lambda_per_hour_cei","lambda_per_hour_t2"]
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
        lambda_per_hour_cei=opts["lambda_per_hour_cei"],
        lambda_per_hour_t2=opts["lambda_per_hour_t2"],
        tau=opts["tau"],
        c_max=opts["c_max"],
        w_max=opts["w_max"],
        buses=opts["buses"],
        outpath=opts["outpath"]
    )
end

parse_and_run()
