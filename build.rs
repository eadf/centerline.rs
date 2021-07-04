#![deny(warnings)]

extern crate version_check as rustc;

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    if let Some(is_feature_flaggable) = rustc::is_feature_flaggable() {
        // enable the "hash_drain_filter" feature if using +nightly
        if is_feature_flaggable {
            println!("cargo:rustc-cfg=feature=\"map_first_last\"");
            println!("cargo:rustc-cfg=feature=\"hash_drain_filter\"");
        }
    }
}
