# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/)
and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2022-03-29

### Fixed

- no_std bug introduced by the wrong way of configuring it.


### Changed

- Removed num_trait and replacet the floating point calculations with fixed-crate


## [0.0.3] - 2022-03-24

### Fixed

- Added num_trait to make the crate truly no_std


### Changed

- Clippy and fmt changes


## [0.0.2] - 2021-01-14

### Fixed

- FeatureSet struct members is public now


### Changed

- Added full initialization helper function


## [0.0.1] - 2021-01-09

Initial release to crates.io. The crate moves into 0.1.x versioning after embedded-hal change.

[1.0.0]: https://github.com/mjaakkol/sgpc3-rs/compare/v0.0.3...v1.0.0

[i5]: https://github.com/mjaakkol/sgpc3-rs/pull/5
