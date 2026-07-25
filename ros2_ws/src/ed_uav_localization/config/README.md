# Simulation field profile

`fields/simulation_arena.yaml` is a synthetic geometry for simulator smoke
runs. It is not measured hardware-field data and its provenance is explicitly
`synthetic_simulation` with `activation: blocked`. The mission executor accepts
it only when launched with `simulation_only:=true`; it cannot activate
competition or hardware flight.
