# Foxglove Extensions

This folder contains custom Foxglove tooling for Banana Hand.

## Quick Start
Install the force-pad extension:
```bash
cd /home/lokesh/BananaHand/software/foxglove/banana-hand-force-heatmap
npm install
npm run local-install
```

Then reload Foxglove, add `BananaHandForceHeatmap`, and set these in panel settings:
- `topic` for example `/rx_force`
- `dataIndex` for example `0`
- `title` for example `Thumb`

## Available Extensions
- `banana-hand-force-heatmap`
  custom panel for a single configurable force pad

## Build
Foxglove extensions require Node.js and npm.

Example:
```bash
cd /home/lokesh/BananaHand/software/foxglove/banana-hand-force-heatmap
npm install
npm run local-install
```

After that, restart or reload the Foxglove desktop app and add the custom panel from the Add Panel menu.
