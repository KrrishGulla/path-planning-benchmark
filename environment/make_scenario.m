function [raw, infl, s, g] = make_scenario(type, P, seed)
%MAKE_SCENARIO  Reproducible map + start/goal for one benchmark trial.
%   raw  : true obstacle map (used for safety/clearance evaluation)
%   infl : inflated C-space map (what every planner plans on)
for attempt = 0:30
    set_seed(seed + 7919 * attempt);
    raw  = generate_map(type, P.mapSize);
    infl = inflate_map(raw, P.inflateRadius);
    [s, g, ok] = sample_start_goal(raw, infl, type, P);
    if ok, return; end
end
error('make_scenario: could not build a solvable "%s" scenario', type);
end
