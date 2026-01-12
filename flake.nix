{
  description = "Robotics Development Environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs { inherit system; };

        applications = with pkgs; [
          nil
          nixd

          temurin-bin
          gradle
          jdt-language-server

		  nodejs
		  prettier
        ];

        libraries = with pkgs; [
          openssl
          gcc
        ];
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = applications ++ libraries;

          OPENSSL_DIR = pkgs.openssl.dev;
          OPENSSL_LIB_DIR = "${pkgs.openssl.out}/lib";
          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath libraries;
          PKG_CONFIG_PATH = pkgs.lib.makeSearchPath "lib/pkgconfig" libraries;
        };
      }
    );
}
