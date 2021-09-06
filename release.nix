{ nixpkgs ? import ./fetchNixpkgs.nix { }
#{ nixpkgs ? import (builtins.fetchTarball https://github.com/airalab/airapkgs/archive/nixos-unstable.tar.gz)
, system ? builtins.currentSystem
}:

let
  pkgs = import nixpkgs { inherit system; };
in rec {
  package = pkgs.callPackage ./default.nix {  };
}
