# Configuration

The shared Robot/bridge template is `src/robot/config/owl_ego.yaml` at the repository root.
`scripts/owl_ego/configure.py` generates a passive workspace configuration and EGO
parameters from the pinned upstream launch. Pass its absolute path to `owl_ego.launch`.
No real camera calibration or enabled flight configuration is shipped here.
