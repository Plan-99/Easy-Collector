import { NextRequest } from "next/server";
import { authenticateLauncher, unauthorized } from "@/lib/launcher-auth";

// Launcher pings this to confirm the saved Bearer token is still valid and
// to refresh user / device metadata on startup.
export async function GET(req: NextRequest) {
  const auth = await authenticateLauncher(req);
  if (!auth) return unauthorized();
  return Response.json({ user: auth.user, device: auth.device });
}
