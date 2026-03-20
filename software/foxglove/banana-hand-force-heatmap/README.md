# Banana Hand Force Heatmap

Custom Foxglove panel extension for visualizing one force value as a compact heat pad.

The panel is designed for:
- raw `/rx_force` array-style messages
- `/banana_hand/force_state`

The topic is selected from the normal Foxglove panel settings sidebar rather than being hardcoded.
Each panel instance lets you choose:
- the source topic
- the array or position index to read
- the panel title
- the min/max color scale

For raw `/rx_force`, the panel reads `message.data` and maps the 10 array values in this order:
- `0` `thumb`
- `1` `index`
- `2` `middle`
- `3` `ring`
- `4` `pinky`
- `5` `palm_1`
- `6` `palm_2`
- `7` `palm_3`
- `8` `palm_4`
- `9` `palm_5`

## What It Shows
- one configurable force value per panel instance

## Install
Foxglove custom panels require Node.js. Foxglove’s docs currently recommend using the extension tooling and then running `npm run local-install` to copy the built extension into your local Foxglove extensions folder. Sources:
- https://docs.foxglove.dev/docs/extensions
- https://docs.foxglove.dev/docs/extensions/local-development
- https://docs.foxglove.dev/docs/extensions/guides/create-custom-panel

On a machine with Node and npm installed:
```bash
cd /home/lokesh/BananaHand/software/foxglove/banana-hand-force-heatmap
npm install
npm run local-install
```

Then restart or reload Foxglove and add the `BananaHandForceHeatmap` panel from the Add Panel menu.

If `npm install` fails, make sure the package versions match Foxglove's current extension toolchain. This project is pinned to:
- `@foxglove/extension` `^2.35.0`
- `create-foxglove-extension` `^1.0.6`

This project's `local-install` script currently maps to:
```bash
npm run build && foxglove-extension install
```

## Use
Start the Banana Hand visualization stack:
```bash
cd /home/lokesh/BananaHand/software/ros
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch banana_hand_visualization mujoco_visualization.launch.py
```

Then in Foxglove:
1. Connect to the Foxglove bridge
2. Add `BananaHandForceHeatmap`
3. Open the panel settings and choose:
   - `/rx_force`
   - the `dataIndex` you want, for example `0` through `9`
   - a custom title, for example `Thumb Tip`
4. Duplicate the panel if you want multiple pads in one layout

Example layout:
- one pad for `thumb`
- one pad for `index`
- one pad for `middle`
- one pad for `ring`
- one pad for `pinky`
- one pad each for `palm_1` through `palm_5`

## Layout Suggestion
For a nice default debugging layout:
- one row of 5 small fingertip pads
- one row of 5 small palm pads
- optional Plot panel for all `/rx_force` values
- optional Raw Messages panel on `/rx_force`

## Notes
- This repo currently does not have `node` or `npm` installed in the checked environment, so the extension source was scaffolded but not built locally here.
- The panel topic is configured in Foxglove panel settings.
- The displayed value comes from `message.data[dataIndex]` for raw `/rx_force`.
- The color scale is adjustable in panel settings through `min` and `max`.
