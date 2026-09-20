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
      libx11
      libxcursor
      libxext
      libxfixes
      libxi
      libxinerama
      libxrandr
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
    octomap
    openssl
    opencv4
    pangolin
    pcl
    pkg-config
    (python3.withPackages (ps: with ps; [ numpy pyyaml ]))
    unzip
    wget
    wayland
    libx11
    libxcursor
    libxext
    libxfixes
    libxi
    libxinerama
    libxrandr
  ];

  shellHook = ''
    export ORB_SLAM3_ROOT="$(pwd)"
    export CMAKE_BUILD_PARALLEL_LEVEL="''${CMAKE_BUILD_PARALLEL_LEVEL:-$(nproc)}"
    export CMAKE_POLICY_VERSION_MINIMUM=3.5
  '';
}
