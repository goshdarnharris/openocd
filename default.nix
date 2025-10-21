{
  stdenv,
  lib,
  fetchurl,
  pkg-config,
  hidapi,
  tcl,
  jimtcl,
  libjaylink,
  libusb1,
  libgpiod_1,

  enableFtdi ? true,
  libftdi1,

  # Allow selection the hardware targets (SBCs, JTAG Programmers, JTAG Adapters)
  extraHardwareSupport ? [ ],
}:
let

  isWindows = stdenv.hostPlatform.isWindows;
  notWindows = !isWindows;

in
stdenv.mkDerivation rec {
  pname = "openocd";
  version = "0.12.0-luminaire";
  src = fetchFromGitHub {
    owner = "goshdarnharris";
    repo = "openocd";
    rev = "e5fe2057f21bae448c3fdb4a0c612492acf15ec3";
    hash = "sha256-ryVHiL6Yhh8r2RA/5uYKd07Jaow3R0Tu+Rl/YEMHWvo=";
  };

  nativeBuildInputs = [
    pkg-config
    tcl
  ];

  buildInputs = [
    libusb1
  ]
  ++ lib.optionals notWindows [
    hidapi
    jimtcl
    libftdi1
    libjaylink
  ]
  ++
    # tracking issue for v2 api changes https://sourceforge.net/p/openocd/tickets/306/
    lib.optional stdenv.hostPlatform.isLinux libgpiod_1;

  configureFlags = [
    "--disable-werror"
    "--enable-jtag_vpi"
    "--enable-remote-bitbang"
    "--enable-amtjtagaccel"
    "--enable-armjtagew"
    "--enable-gw16012"
    "--enable-jlink"
    "--enable-oocd_trace"
    "--enable-opendous"
    "--enable-osbdm"
    "--enable-parport"
    "--enable-presto_libftdi"
    "--enable-remote-bitbang"
    "--enable-rlink"
    "--enable-stlink"
    "--enable-ti-icdi"
    "--enable-ulink"
    "--enable-usbprog"
    "--enable-vsllink"
    "--enable-aice"
    "--enable-cmsis-dap"
    "--enable-dummy"
    "--enable-jtag_vpi"
    "--enable-openjtag_ftdi"
    "--enable-usb-blaster-2"
    "--enable-usb_blaster_libftdi"
    (lib.enableFeature notWindows "buspirate")
    (lib.enableFeature (notWindows && enableFtdi) "ftdi")
    (lib.enableFeature stdenv.hostPlatform.isLinux "linuxgpiod")
    (lib.enableFeature stdenv.hostPlatform.isLinux "sysfsgpio")
    (lib.enableFeature isWindows "internal-jimtcl")
    (lib.enableFeature isWindows "internal-libjaylink")
  ]
  ++ map (hardware: "--enable-${hardware}") extraHardwareSupport;

  enableParallelBuilding = true;

  env.NIX_CFLAGS_COMPILE = toString (
    lib.optionals stdenv.cc.isGNU [
      "-Wno-error=cpp"
      "-Wno-error=strict-prototypes" # fixes build failure with hidapi 0.10.0
    ]
  );

  postInstall = lib.optionalString stdenv.hostPlatform.isLinux ''
    mkdir -p "$out/etc/udev/rules.d"
    rules="$out/share/openocd/contrib/60-openocd.rules"
    if [ ! -f "$rules" ]; then
        echo "$rules is missing, must update the Nix file."
        exit 1
    fi
    ln -s "$rules" "$out/etc/udev/rules.d/"
  '';

  meta = with lib; {
    description = "Free and Open On-Chip Debugging, In-System Programming and Boundary-Scan Testing";
    mainProgram = "openocd";
    longDescription = ''
      OpenOCD provides on-chip programming and debugging support with a layered
      architecture of JTAG interface and TAP support, debug target support
      (e.g. ARM, MIPS), and flash chip drivers (e.g. CFI, NAND, etc.).  Several
      network interfaces are available for interactiving with OpenOCD: HTTP,
      telnet, TCL, and GDB.  The GDB server enables OpenOCD to function as a
      "remote target" for source-level debugging of embedded systems using the
      GNU GDB program.
    '';
    homepage = "https://openocd.sourceforge.net/";
    license = licenses.gpl2Plus;
    maintainers = with maintainers; [
      bjornfor
      prusnak
    ];
    platforms = platforms.unix ++ platforms.windows;
  };
}