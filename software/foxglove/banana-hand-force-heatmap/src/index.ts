import { ExtensionContext } from "@foxglove/extension";

import { initBananaHandForceHeatmapPanel } from "./panel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "BananaHandForceHeatmap",
    initPanel: initBananaHandForceHeatmapPanel,
  });
}
