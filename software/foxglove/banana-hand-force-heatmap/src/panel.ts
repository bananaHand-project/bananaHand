import { PanelExtensionContext } from "@foxglove/extension";

type PanelState = {
  topic: string;
  dataIndex: number;
  title: string;
  min: number;
  max: number;
};

const DEFAULT_TOPIC = "";
const DEFAULT_INDEX = 0;
const DEFAULT_TITLE = "Force Pad";
const DEFAULT_MIN = 0;
const DEFAULT_MAX = 1200;

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function colorForValue(value: number, min: number, max: number): string {
  const ratio = clamp((value - min) / Math.max(1, max - min), 0, 1);
  const hue = 210 - ratio * 210;
  const saturation = 88;
  const lightness = 58 - ratio * 18;
  return `hsl(${hue}, ${saturation}%, ${lightness}%)`;
}

function borderForValue(value: number, min: number, max: number): string {
  const ratio = clamp((value - min) / Math.max(1, max - min), 0, 1);
  return `rgba(255, 255, 255, ${0.14 + ratio * 0.36})`;
}

function extractIndexedValue(message: unknown, dataIndex: number): number | undefined {
  if (typeof message !== "object" || message == undefined) {
    return undefined;
  }

  const maybeJointState = message as {
    position?: unknown;
  };
  if (Array.isArray(maybeJointState.position) && dataIndex < maybeJointState.position.length) {
    const value = maybeJointState.position[dataIndex];
    if (typeof value === "number" && Number.isFinite(value)) {
      return value;
    }
  }

  const maybeArray = message as { data?: unknown };
  if (Array.isArray(maybeArray.data) || ArrayBuffer.isView(maybeArray.data)) {
    const rawValues = Array.from(maybeArray.data as ArrayLike<unknown>);
    if (dataIndex < rawValues.length) {
      const value = rawValues[dataIndex];
      if (typeof value === "number" && Number.isFinite(value)) {
        return value;
      }
    }
  }

  return undefined;
}

export function initBananaHandForceHeatmapPanel(context: PanelExtensionContext): () => void {
  const initialState = (context.initialState as Partial<PanelState> | undefined) ?? {};

  let topic = initialState.topic ?? DEFAULT_TOPIC;
  let dataIndex = initialState.dataIndex ?? DEFAULT_INDEX;
  let titleText = initialState.title ?? DEFAULT_TITLE;
  let minValue = initialState.min ?? DEFAULT_MIN;
  let maxValue = initialState.max ?? DEFAULT_MAX;
  let currentValue = 0;
  let hasSeenData = false;
  let availableTopics: string[] = [];

  context.panelElement.innerHTML = "";
  context.panelElement.style.margin = "0";
  context.panelElement.style.background = "#09111f";
  context.panelElement.style.color = "#ebf2ff";
  context.panelElement.style.fontFamily =
    "ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, sans-serif";

  const root = document.createElement("div");
  root.style.height = "100%";
  root.style.display = "grid";
  root.style.gridTemplateRows = "auto auto 1fr";
  root.style.gap = "10px";
  root.style.padding = "10px";

  const title = document.createElement("div");
  title.style.display = "grid";
  title.style.gap = "4px";

  const heading = document.createElement("div");
  heading.style.fontSize = "14px";
  heading.style.fontWeight = "700";
  heading.textContent = titleText;

  const subheading = document.createElement("div");
  subheading.style.fontSize = "11px";
  subheading.style.opacity = "0.72";

  const pad = document.createElement("div");
  pad.style.height = "100%";
  pad.style.minHeight = "96px";
  pad.style.borderRadius = "16px";
  pad.style.border = "1px solid rgba(255,255,255,0.12)";
  pad.style.padding = "10px";
  pad.style.display = "grid";
  pad.style.alignContent = "space-between";
  pad.style.boxShadow = "inset 0 0 0 1px rgba(255,255,255,0.02)";

  const padLabel = document.createElement("div");
  padLabel.style.fontSize = "11px";
  padLabel.style.textTransform = "uppercase";
  padLabel.style.letterSpacing = "0.08em";
  padLabel.style.opacity = "0.82";

  const valueLabel = document.createElement("div");
  valueLabel.style.fontSize = "34px";
  valueLabel.style.fontWeight = "700";
  valueLabel.textContent = "--";

  pad.appendChild(padLabel);
  pad.appendChild(valueLabel);
  title.appendChild(heading);
  title.appendChild(subheading);
  root.appendChild(title);
  root.appendChild(pad);
  context.panelElement.appendChild(root);

  function savePanelState(): void {
    context.saveState({
      topic,
      dataIndex,
      title: titleText,
      min: minValue,
      max: maxValue,
    });
  }

  function subscribeToTopic(): void {
    context.unsubscribeAll();
    if (topic) {
      context.subscribe([{ topic }]);
      context.setDefaultPanelTitle(titleText || `Force Pad (${dataIndex})`);
    } else {
      context.setDefaultPanelTitle(titleText || "Force Pad");
    }
  }

  function updateSettingsEditor(): void {
    context.updatePanelSettingsEditor({
      enableFilter: true,
      nodes: {
        general: {
          label: "General",
          fields: {
            title: {
              label: "Title",
              input: "string",
              value: titleText,
              placeholder: "Thumb tip",
            },
            topic: {
              label: "Topic",
              input: "autocomplete",
              items: availableTopics,
              value: topic,
              placeholder: "/rx_force",
              help: "Select the incoming force topic.",
              error: topic && !availableTopics.includes(topic) ? "Topic is not currently available" : undefined,
            },
            dataIndex: {
              label: "Data Index",
              input: "number",
              value: dataIndex,
              min: 0,
              step: 1,
              help: "Array index in message.data or position index in JointState.",
            },
            min: {
              label: "Min",
              input: "number",
              value: minValue,
            },
            max: {
              label: "Max",
              input: "number",
              value: maxValue,
            },
          },
        },
      },
      actionHandler: (action: any) => {
        if (action.action !== "update") {
          return;
        }

        const path = action.payload.path as string[];
        const value = action.payload.value;
        if (path[0] !== "general") {
          return;
        }

        if (path[1] === "title" && typeof value === "string") {
          titleText = value;
          savePanelState();
          subscribeToTopic();
          updateSettingsEditor();
          renderPad();
          return;
        }

        if (path[1] === "topic" && typeof value === "string") {
          topic = value;
          hasSeenData = false;
          currentValue = 0;
          savePanelState();
          subscribeToTopic();
          updateSettingsEditor();
          renderPad();
          return;
        }

        if (path[1] === "dataIndex" && typeof value === "number") {
          dataIndex = Math.max(0, Math.floor(value));
          hasSeenData = false;
          currentValue = 0;
          savePanelState();
          updateSettingsEditor();
          renderPad();
          return;
        }

        if (path[1] === "min" && typeof value === "number") {
          minValue = value;
          savePanelState();
          renderPad();
          return;
        }

        if (path[1] === "max" && typeof value === "number") {
          maxValue = value;
          savePanelState();
          renderPad();
        }
      },
    });
  }

  function renderPad(): void {
    heading.textContent = titleText || DEFAULT_TITLE;
    padLabel.textContent = topic ? `index ${dataIndex}` : "no topic";

    if (!topic) {
      subheading.textContent = "Choose a topic in panel settings";
      valueLabel.textContent = "--";
      pad.style.background = "#10192c";
      pad.style.borderColor = "rgba(255,255,255,0.12)";
      return;
    }

    subheading.textContent = hasSeenData
      ? `Live data from ${topic}`
      : `Waiting for data on ${topic}`;
    valueLabel.textContent = hasSeenData ? currentValue.toFixed(0) : "--";
    pad.style.background = colorForValue(currentValue, minValue, maxValue);
    pad.style.borderColor = borderForValue(currentValue, minValue, maxValue);
  }

  subscribeToTopic();
  updateSettingsEditor();
  context.watch("currentFrame");
  context.watch("topics");

  context.onRender = (renderState, done) => {
    const nextTopics = (renderState.topics ?? []).map((entry) => entry.name).sort();
    if (
      nextTopics.length !== availableTopics.length ||
      nextTopics.some((item, index) => item !== availableTopics[index])
    ) {
      availableTopics = nextTopics;
      updateSettingsEditor();
    }

    const frame = renderState.currentFrame;
    if (frame && topic) {
      for (let i = frame.length - 1; i >= 0; i -= 1) {
        const event = frame[i] as { topic?: string; message?: unknown };
        if (event.topic !== topic) {
          continue;
        }

        const nextValue = extractIndexedValue(event.message, dataIndex);
        if (nextValue != undefined) {
          currentValue = nextValue;
          hasSeenData = true;
          break;
        }
      }
    }

    renderPad();
    done();
  };

  renderPad();

  return () => {
    context.panelElement.innerHTML = "";
  };
}
