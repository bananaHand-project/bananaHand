# Foxglove Extensions

This folder contains custom Foxglove tooling for Banana Hand.

## Requirements
- Node.js 20+
- npm 10+

Check your versions:
```bash
node -v
npm -v
```

If you see an error like `Unexpected token '?'` during `foxglove-extension build`, your Node version is too old.

## Quick Start
Install the force-pad extension:
```bash
cd software/foxglove/banana-hand-force-heatmap
# optional if you use nvm:
# nvm use
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
cd software/foxglove/banana-hand-force-heatmap
# optional if you use nvm:
# nvm use
npm install
npm run local-install
```

After that, restart or reload the Foxglove desktop app and add the custom panel from the Add Panel menu.
