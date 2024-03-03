include "mapping.summitxl.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 9,
}
POSE_GRAPH.optimize_every_n_nodes = 100

return options
