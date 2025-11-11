using Random
using JuMP
using JSON
using CPLEX
using Dates

Random.seed!(42)

function parse_range(x)
    if x isa AbstractVector && length(x) == 2
        return Int(x[1]):Int(x[2])
    elseif x isa String
        m = match(r"^\s*(\d+)\s*:\s*(\d+)\s*$", x)
        if m !== nothing
            a, b = parse(Int, m.captures[1]), parse(Int, m.captures[2])
            return a:b
        end
    elseif x isa Integer
        return 1:Int(x)
    elseif x isa AbstractVector
        v = collect(Int.(x))
        if !isempty(v) && maximum(v) - minimum(v) + 1 == length(v)
            return minimum(v):maximum(v)
        else
            return v
        end
    end
    error("Unsupported range format: $x")
end

function parse_arrivals(d)::Dict{Int,Int}
    res = Dict{Int,Int}()
    for (k, v) in d
        kk = k isa Integer ? Int(k) : parse(Int, String(k))
        res[kk] = Int(v)
    end
    return res
end

function load_config(path::AbstractString)
    cfg = JSON.parsefile(path)
    getk(k, default=nothing) = haskey(cfg, k) ? cfg[k] : default

    T       = parse_range(getk("T", "0:780"))
    B       = parse_range(getk("B", 1))
    P_raw   = getk("P", nothing)
    P_r_raw = getk("P_r", nothing)
    L       = Int(getk("L", 1))
    tau     = Int(getk("tau", 25))
    c_max   = Int(getk("c_max", 5))
    w_max   = Int(getk("w_max", 60))

    arr   = haskey(cfg, "arrivals") && haskey(cfg["arrivals"], "CEI") ? parse_arrivals(cfg["arrivals"]["CEI"]) : Dict{Int,Int}()
    arr_r = haskey(cfg, "arrivals") && haskey(cfg["arrivals"], "T2")  ? parse_arrivals(cfg["arrivals"]["T2"])  : Dict{Int,Int}()

    P   = P_raw   === nothing ? (isempty(arr)   ? (1:0) : (minimum(keys(arr))  : maximum(keys(arr))))  : parse_range(P_raw)
    P_r = P_r_raw === nothing ? (isempty(arr_r) ? (1:0) : (minimum(keys(arr_r)) : maximum(keys(arr_r)))) : parse_range(P_r_raw)

    # initpos = getk("initial_positions", Any[])

    # return (; T, B, P, P_r, L, tau, c_max, w_max, arr, arr_r, initpos)
    return (; T, B, P, P_r, L, tau, c_max, w_max, arr, arr_r)
end

cfg_path   = length(ARGS) >= 1 ? ARGS[1] : ""
out_result = length(ARGS) >= 2 ? ARGS[2] : "result.json"

cfg = cfg_path == "" ? nothing : load_config(cfg_path)

if cfg != nothing
    cfg = load_config(ARGS[1])
    # T, B = cfg.T, cfg.B
    T, B, P, P_r = cfg.T, cfg.B, cfg.P, cfg.P_r
    L, tau, c_max, w_max = cfg.L, cfg.tau, cfg.c_max, cfg.w_max
    arr, arr_r = cfg.arr, cfg.arr_r
    # initial_positions = cfg.initpos
else
    #+ Default values
    T      = 0:8
    B      = 1:1
    P      = 1:2
    P_r    = 1:1
    L      = 1
    tau    = 2
    c_max  = 1
    w_max  = 60
    arr    = Dict(1 => 0, 2 => 0)
    arr_r  = Dict(1 => 2)
    # initial_positions = Any[]
end

println("T=$(first(T)):$(last(T))  B=$B  P=$P  P_r=$P_r  L=$L  tau=$tau  c_max=$c_max  w_max=$w_max")
println("Output file: $out_result")
println("arr(CEI) = ", arr)
println("arr(T2 ) = ", arr_r)

# --------- Build feasible time windows per passenger ---------
Tmax_board = last(T) - tau

T_i  = Dict{Int, Vector{Int}}()
for i in P
    lo = get(arr, i, typemax(Int))
    hi = min(lo + w_max, Tmax_board)
    T_i[i] = (lo <= hi) ? collect(lo:hi) : Int[]
end

T_ir = Dict{Int, Vector{Int}}()
for i in P_r
    lo = get(arr_r, i, typemax(Int))
    hi = min(lo + w_max, Tmax_board)
    T_ir[i] = (lo <= hi) ? collect(lo:hi) : Int[]
end

println("====================================")

model = Model(CPLEX.Optimizer)
set_optimizer_attribute(model, "CPX_PARAM_THREADS", 20)
set_optimizer_attribute(model, "CPX_PARAM_MIPEMPHASIS", 1) # feasibility
set_optimizer_attribute(model, "CPX_PARAM_HEURFREQ", 10)
# set_optimizer_attribute(model, "CPXPARAM_MIP_Strategy_File", 3)
# set_optimizer_attribute(model, "CPX_PARAM_NODEFILEIND", 3)
set_string_names_on_creation(model, false)
set_optimizer_attribute(model, "CPXPARAM_MIP_Tolerances_MIPGap", 0.04) # gap 4 %
set_optimizer_attribute(model, "CPX_PARAM_TILIM", 259200.0) # 3 days

# Decision variables

@variable(model, x[i in P, c in B, t in T_i[i]], Bin)     # CEI -> CRBT2
@variable(model, x_r[i in P_r, c in B, t in T_ir[i]], Bin) # CRBT2 -> CEI

# Bus availability and departure decisions
@variable(model, ba[t in T, c in B],   Bin)   # bus available at CEI at time t
@variable(model, ba_r[t in T, c in B], Bin)   # bus available at CRBT2 at time t
@variable(model, bd[t in T, c in B],   Bin)   # depart from CEI at t
@variable(model, bd_r[t in T, c in B], Bin)   # depart from CRBT2 at t

# Objective: minimize total waiting time (no Big-M, no Dep, no Z)
@objective(model, Min,
    sum(((t - arr[i])^2)   * x[i,c,t]   for i in P,   c in B, t in T_i[i]) +
    sum(((t - arr_r[i])^2) * x_r[i,c,t] for i in P_r, c in B, t in T_ir[i])
)

# 1) Each passenger assigned exactly once
@constraint(model, [i in P],   sum(x[i,c,t]   for c in B, t in T_i[i])   == 1)
@constraint(model, [i in P_r], sum(x_r[i,c,t] for c in B, t in T_ir[i]) == 1)

# 2) Capacity and gating by departures (remove redundant caps)
@constraint(model, [t in T, c in B], sum(x[i,c,t]   for i in P if t in T_i[i])   <= c_max * bd[t,c])
@constraint(model, [t in T, c in B], sum(x_r[i,c,t] for i in P_r if t in T_ir[i]) <= c_max * bd_r[t,c])

# Link assignment to departures (x ≤ bd)
@constraint(model, [i in P,   c in B, t in T_i[i]],   x[i,c,t]   <= bd[t,c])
@constraint(model, [i in P_r, c in B, t in T_ir[i]], x_r[i,c,t] <= bd_r[t,c])

# 3) Bus must exist at terminal to depart
@constraint(model, [t in T, c in B], bd[t,c]   <= ba[t,c])
@constraint(model, [t in T, c in B], bd_r[t,c] <= ba_r[t,c])

# 4) Cannot be at both terminals simultaneously
@constraint(model, [t in T, c in B], ba[t,c] + ba_r[t,c] <= 1)

# 5) Start-of-day: bus available at exactly one terminal
@constraint(model, [c in B], ba[first(T), c] + ba_r[first(T), c] == 1)

# 6) No boarding in the last τ time slots: already excluded via T_i/T_ir upper bound
#    But still forbid bus departures that cannot complete trip if desired (optional).
#    If you want to forbid departures after Tmax_board at both terminals:
#for t in (Tmax_board+1):last(T), c in B
#    fix(bd[t,c], 0.0; force=true)
#    fix(bd_r[t,c], 0.0; force=true)
#end

#! if there are no passenger at time bus not depart
#! new (Not test yet)
# for t in T, c in B
#     if all(!(t in T_i[p]) for p in P)
#         fix(bd[t,c], 0.0; force=true)
#     end

#     if all(!(t in T_ir[p]) for p in P_r)
#         fix(bd_r[t,c], 0.0; force=true)
#     end
# end

#! have to comment but if it commented the process got killed
# for t in T, c in B
#     if t % tau != 0
#         fix(bd[t,c], 0.0; force=true)
#         fix(bd_r[t,c], 0.0; force=true)
#     end
# end

# 7) Flow equalities of bus state (same logic as before)
@constraint(model, [t in T, c in B], bd[t,c] + bd_r[t,c] <= 1)

for c in B
    for t in T[1:end-1]   # every t < last(T)
        if t >= tau
            @constraint(model, ba[t, c]   == ba[t - 1, c]   - bd[t - 1, c]   + bd_r[t - tau, c])
            @constraint(model, ba_r[t, c] == ba_r[t - 1, c] - bd_r[t - 1, c] + bd[t - tau, c])

        """
        else
            @constraint(model, ba[t+1, c]   == ba[t, c]   - bd[t, c])
            @constraint(model, ba_r[t+1, c] == ba_r[t, c] - bd_r[t, c])
        """

        end
    end
end

for c in B
    for t in 1:tau-1
        @constraint(model, ba[t,c]   == ba[t-1,c] - bd[t-1,c])
        @constraint(model, ba_r[t,c] == ba_r[t-1,c] - bd_r[t-1,c])
    end
end


# 8) At each terminal, buses must depart at least two minutes apart
for t in T[1:end-2]
    @constraint(model, sum(bd[t,c]  + bd[t+1, c] + bd[t+2, c] for c in B) <= 1)
    @constraint(model, sum(bd_r[t,c]  + bd_r[t+1, c] + bd_r[t+2, c] for c in B)  <= 1)
end

optimize!(model)
println("====================================")
println("\nStatus: ", termination_status(model))
if termination_status(model) == MOI.OPTIMAL || termination_status(model) == MOI.FEASIBLE_POINT
    println("Objective value (total waiting): ", objective_value(model))

    println("\nDepartures from CEI (bd[t]=1):")
    for t in T, c in B
        if value(bd[t,c]) > 0.5
            println("  t=$t, c=$c bd=", value(bd[t,c]))
        end
    end

    println("\nDepartures from CRBT2 (bd_r[t]=1):")
    for t in T, c in B
        if value(bd_r[t,c]) > 0.5
            println("  t=$t, c=$c bd_r=", value(bd_r[t,c]))
        end
    end

    println("\nSample assignments (first 10 at CEI):")
    let shown = 0
        for i in P, c in B, t in T_i[i]
            if value(x[i,c,t]) > 0.5
                wait_i = t - arr[i]
                println("  i=$i -> t=$t (CEI), wait=", wait_i)
                shown += 1
                if shown >= 20
                    break
                end
            end
        end
    end

    println("\nSample assignments (CRBT2):")
    for i in P_r, c in B, t in T_ir[i]
        if value(x_r[i,c,t]) > 0.5
            wait_ir = t - arr_r[i]
            println("  i_r=$i -> t=$t (CRBT2), wait=", wait_ir)
        end
    end

    println("\nInitial availability:")
    for c in B
        println("  ba[0,$c]   = ", value(ba[first(T),c]))
        println("  ba_r[0,$c] = ", value(ba_r[first(T),c]))
    end
else
    println("Model not optimal. Consider loosening w_max or expanding T.")
end

function result_payload()
    # meta
    meta = Dict(
        "T_start" => first(T), "T_end" => last(T), "L" => L,
        "tau" => tau, "capacity" => c_max, "w_max" => w_max
    )

    # sets
    sets = Dict(
        "buses" => collect(B),
        "P_CEI" => collect(P),
        "P_T2"  => collect(P_r)
    )

    # arrivals
    arr_cei = [Dict("p"=>i, "arr"=>arr[i])   for i in P]
    arr_t2  = [Dict("p"=>i, "arr"=>arr_r[i]) for i in P_r]

    # initial positions from ba[0,*] / ba_r[0,*]
    # initpos = Vector{Dict{String,Any}}()
    # for c in B
    #     b0  = value(ba[first(T), c])
    #     b0r = value(ba_r[first(T), c])
    #     if b0 > 0.5 && b0r < 0.5
    #         push!(initpos, Dict("bus"=>c, "terminal"=>"CEI"))
    #     elseif b0r > 0.5 && b0 < 0.5
    #         push!(initpos, Dict("bus"=>c, "terminal"=>"T2"))
    #     else
    #         # หากโมเดลกำหนดไม่ชัด ให้ fallback จาก constraint เริ่มวัน
    #         term = b0 >= b0r ? "CEI" : "T2"
    #         push!(initpos, Dict("bus"=>c, "terminal"=>term))
    #     end
    # end

    # departures (collect only bd==1 / bd_r==1)
    deps = Vector{Dict{String,Any}}()
    for t in T, c in B
        if value(bd[t,c]) > 0.5
            push!(deps, Dict("terminal"=>"CEI", "bus"=>c, "t"=>t))
        end
        if value(bd_r[t,c]) > 0.5
            push!(deps, Dict("terminal"=>"T2", "bus"=>c, "t"=>t))
        end
    end

    # assignments (x==1 / x_r==1)
    asg = Vector{Dict{String,Any}}()
    for i in P, c in B, t in T_i[i]
        if value(x[i,c,t]) > 0.5
            push!(asg, Dict("terminal"=>"CEI", "p"=>i, "bus"=>c, "t"=>t, "wait"=>(t - arr[i])))
        end
    end
    for i in P_r, c in B, t in T_ir[i]
        if value(x_r[i,c,t]) > 0.5
            push!(asg, Dict("terminal"=>"T2", "p"=>i, "bus"=>c, "t"=>t, "wait"=>(t - arr_r[i])))
        end
    end

    return Dict(
        "schema_version" => 1,
        "meta" => meta,
        "generated_at" => string(Dates.now()+Dates.Hour(7)),
        "status" => string(termination_status(model)),
        "objective" => (isfinite(objective_value(model)) ? objective_value(model) : nothing),
        "sets" => sets,
        "arrivals" => Dict("CEI"=>arr_cei, "T2"=>arr_t2),
        # "initial_positions" => initpos,
        "departures" => deps,
        "assignments" => asg
    )
end

if termination_status(model) in (MOI.OPTIMAL, MOI.FEASIBLE_POINT, MOI.TIME_LIMIT) && primal_status(model) == MOI.FEASIBLE_POINT
    payload = result_payload()
    # Dump as File
    open(out_result, "w") do io
        write(io, JSON.json(payload))  # one-line JSON
    end
    println("Wrote result to $out_result")

else
    # if infeasible unless raise meta+status for checking
    payload = Dict(
        "schema_version"=>1,
        "meta"=>Dict("T_start"=>first(T),"T_end"=>last(T),"L"=>L,"tau"=>tau,"capacity"=>c_max,"w_max"=>w_max),
        "generated_at" => string(Dates.now()+Dates.Hour(7)),
        "status"=>string(termination_status(model)),
        "objective"=>nothing,
        "sets"=>Dict("buses"=>collect(B),"P_CEI"=>collect(P),"P_T2"=>collect(P_r)),
        "arrivals"=>Dict("CEI"=>[], "T2"=>[]),
        "initial_positions"=>[], "departures"=>[], "assignments"=>[]
    )
    open(out_result,"w") do io; write(io, JSON.json(payload)); end
    println("Wrote result to $out_result")
end
