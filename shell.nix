{ pkgs ? import <nixpkgs> {} }:
let
  pangolin = pkgs.stdenv.mkDerivation {
    pname = "pangolin";
    version = "0.9.2";
    src = pkgs.fetchFromGitHub {
      owner = "stevenlovegrove";
      repo = "Pangolin";
      rev = "v0.9.2";
      hash = "sha256-rRR/+PdVcnOiv0X7OGBmTQe5l1XQ94nzyy02Tf95AWQ=";
    };

    nativeBuildInputs = with pkgs; [ cmake pkg-config ];
    buildInputs = with pkgs; [
      eigen
      libGL
      libepoxy
      xorg.libX11
      xorg.libXcursor
      xorg.libXext
      xorg.libXfixes
      xorg.libXi
      xorg.libXinerama
      xorg.libXrandr
    ];

    cmakeFlags = [
      "-DBUILD_EXAMPLES=OFF"
      "-DBUILD_TOOLS=OFF"
      "-DBUILD_TESTS=OFF"
      "-DBUILD_PANGOLIN_PYTHON=OFF"
    ];
  };
in
pkgs.mkShell {
  name = "orb-slam3";
  packages = with pkgs; [
    boost
    cmake
    eigen
    git
    glew
    gnumake
    libGL
    libGLU
    libepoxy
    librealsense
    libxkbcommon
    mesa
    openssl
    opencv4
    pangolin
    pkg-config
    python3
    unzip
    wget
    wayland
    xorg.libX11
    xorg.libXcursor
    xorg.libXext
    xorg.libXfixes
    xorg.libXi
    xorg.libXinerama
    xorg.libXrandr
  ];

  shellHook = ''
    export ORB_SLAM3_ROOT="$(pwd)"
    export CMAKE_BUILD_PARALLEL_LEVEL="''${CMAKE_BUILD_PARALLEL_LEVEL:-$(nproc)}"
  '';
}
