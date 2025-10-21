{
  description = "A Nix-flake-based C/C++ development environment";

  inputs =
    {
      nixpkgs.url = "https://flakehub.com/f/NixOS/nixpkgs/0"; # stable Nixpkgs
      flake-utils.url = "github:numtide/flake-utils";
    };

  outputs =
    { self, ... }@inputs:

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
      devShells = forEachSupportedSystem (
        { pkgs }:
        {
          default =
            pkgs.mkShell
              {
                packages =
                  with pkgs;
                  [
                    gcc-arm-embedded-13
                    pkgs.llvmPackages_21.clangNoLibc
                    libtool
                    automake
                  ];
              };
        }
      );
    };
}