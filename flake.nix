{
  description = "A Nix-flake-based C/C++ development environment";

  inputs =
    {
      nixpkgs.url = "https://flakehub.com/f/NixOS/nixpkgs/0"; # stable Nixpkgs
      flake-utils.url = "github:numtide/flake-utils";
    };

  outputs =
    { self, nixpkgs, ... }@inputs:

    let
      supportedSystems = [
        "x86_64-linux"
        "aarch64-linux"
      ];
      forEachSupportedSystem =
        f:
        inputs.nixpkgs.lib.genAttrs supportedSystems (
          system:
          f {
            pkgs = import inputs.nixpkgs { inherit system; };
          }
        );
    in
    {
      packages.x86_64-linux.default = 
        with import nixpkgs { system = "x86_64-linux"; };
        pkgs.callPackage ./package.nix { };      
    };
}