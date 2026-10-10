import { StrictMode } from "react";
import { createRoot } from "react-dom/client";

import { App } from "./App";
import { RosbridgeClient } from "./lib/client";
import { rosbridgeUrl } from "./lib/url";
import "./styles.css";

const client = new RosbridgeClient(rosbridgeUrl(window.location));
const root = document.getElementById("root");
if (!root) {
  throw new Error("missing #root element");
}
createRoot(root).render(
  <StrictMode>
    <App client={client} />
  </StrictMode>,
);
