/** Where to find rosbridge: `?ros=ws://host:port`, otherwise port 9090 on the page's host. */
export function rosbridgeUrl(location: Pick<Location, "hostname" | "search" | "protocol">): string {
  const override = new URLSearchParams(location.search).get("ros");
  if (override) {
    return override;
  }
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  return `${scheme}://${location.hostname || "localhost"}:9090`;
}
