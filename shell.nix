{
  pkgs ? import <nixpkgs> { },
}:

let
  frc-nix = builtins.getFlake "github:frc4451/frc-nix";
in
(pkgs.buildFHSEnv {
  name = "robotics-dev";
  targetPkgs =
    pkgs:
    (with pkgs; [
      nil
      nixd
      nixfmt

      javaPackages.compiler.temurin-bin.jdk-17
      gradle
      jdt-language-server

      nodejs
      prettier

      openssl
      gcc
    ]);

  multiPkgs =
    pkgs:
    (with pkgs; [
      openssl
      gcc
    ]);

  runScript = "bash";
}).env
