import NextAuth from "next-auth";
import Google from "next-auth/providers/google";
import Credentials from "next-auth/providers/credentials";
import { PrismaAdapter } from "@auth/prisma-adapter";
import bcrypt from "bcryptjs";
import { prisma } from "@/lib/prisma";

// Credentials provider requires JWT session strategy. Existing DB-backed
// sessions become invalid on the switch — users will need to sign in once
// more via Google, after which a JWT is issued.
export const { handlers, auth, signIn, signOut } = NextAuth({
  adapter: PrismaAdapter(prisma),
  session: { strategy: "jwt" },
  providers: [
    Google,
    Credentials({
      name: "Email",
      credentials: {
        email: { label: "이메일", type: "email" },
        password: { label: "비밀번호", type: "password" },
      },
      authorize: async creds => {
        const email = String(creds?.email || "").trim().toLowerCase();
        const password = String(creds?.password || "");
        if (!email || !password) return null;
        const user = await prisma.user.findUnique({ where: { email } });
        if (!user || !user.passwordHash) return null;
        const ok = await bcrypt.compare(password, user.passwordHash);
        if (!ok) return null;
        return {
          id: user.id,
          email: user.email,
          name: user.name,
          image: user.image,
        };
      },
    }),
  ],
  pages: {
    signIn: "/auth/signin",
  },
  events: {
    async createUser({ user }) {
      // Launch promo: all new accounts get the Unlimited plan automatically.
      // Switch to plan: "free" later when promo ends.
      const adminEmails = (process.env.ADMIN_EMAILS || "")
        .split(",")
        .map(e => e.trim().toLowerCase())
        .filter(Boolean);
      const isAdmin =
        !!user.email && adminEmails.includes(user.email.toLowerCase());
      await prisma.user.update({
        where: { id: user.id! },
        data: {
          plan: "unlimited",
          ...(isAdmin ? { role: "admin" } : {}),
        },
      });
      if (isAdmin) console.log(`[AUTH] Admin role assigned to ${user.email}`);
    },
  },
  callbacks: {
    async jwt({ token, user }) {
      if (user) {
        token.userId = user.id;
      }
      if (token.userId && !token.role) {
        const dbUser = await prisma.user.findUnique({
          where: { id: String(token.userId) },
          select: { role: true },
        });
        token.role = dbUser?.role || "user";
      }
      return token;
    },
    async session({ session, token }) {
      if (session.user && token.userId) {
        session.user.id = String(token.userId);
        (session.user as { role?: string }).role =
          (token.role as string) || "user";
      }
      return session;
    },
  },
});
