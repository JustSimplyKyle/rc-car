{
  description = "ESP32 Rust Dev Shell with Sysroot Fix";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    esp-rs-nix.url = "github:leighleighleigh/esp-rs-nix";
  };

  outputs = { self, nixpkgs, flake-utils, esp-rs-nix }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        upstreamShell = esp-rs-nix.devShells.${system}.default;
        espPkg = esp-rs-nix.packages.${system}.esp-rs;
      in
      {
        devShells.default = pkgs.mkShell {
          inputsFrom = [ upstreamShell ];

          # CARGO_BUILD_TARGET = "xtensa-esp32s3-none-elf";
        
          shellHook = ''
            # Run upstream hook (sets RUSTUP_TOOLCHAIN)
            ${upstreamShell.shellHook}

            # --- FIX START ---
            # Create a local wrapper for rustc. 
            # This forces the compiler to use 'esp-rs' (with sources) as sysroot 
            # instead of resolving to 'esp-rust-build' (without sources).
            
            mkdir -p .local_esp_wrappers
            
            cat <<EOF > .local_esp_wrappers/rustc
            #!/bin/sh
            # Call the rustc binary but force the sysroot to the valid toolchain path
            exec "$RUSTUP_TOOLCHAIN/bin/rustc" --sysroot "$RUSTUP_TOOLCHAIN" "\$@"
            EOF
            
            chmod +x .local_esp_wrappers/rustc
            
            # Instruct Cargo to use our wrapper instead of the raw binary
            export RUSTC="$(pwd)/.local_esp_wrappers/rustc"
            
            # Explicitly set source path as a fallback for other tools (rust-analyzer)
            export RUST_SRC_PATH="${espPkg}/lib/rustlib/src/rust/library"
            # --- FIX END ---
            
            echo "ESP32 Environment Loaded. Compiler wrapper active."
          '';
        };
      }
    );
}
