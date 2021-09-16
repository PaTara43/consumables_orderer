# Consumables Orderer Gaka-Chu model
This is a consumables orderer node for [Gaka-Chu](https://github.com/airalab/robot_painter/tree/test_branch).

Listens to topic `/film`, counts canvases left. When 1, published demand to IPFS pubsub channel.

- to build: `nix build -f release.nix`
- needs `ipfs daemon --enable-pubsub-experiment`; [robonomics_comm](https://github.com/airalab/robonomics_comm) (`./robonomics_comm/robonomics_liability/launch/liability.launch` and `ethereum_common erc20.launch`)
- use `create_objective.py` to create custom objectives
- Number of canvases left: `consumables_remains.txt`
