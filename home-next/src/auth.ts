import NextAuth from "next-auth";
import Google from "next-auth/providers/google";
import { PrismaAdapter } from "@auth/prisma-adapter";
import { prisma } from "@/lib/prisma";
import { v4 as uuidv4 } from "uuid";

export const { handlers, auth, signIn, signOut } = NextAuth({
  adapter: PrismaAdapter(prisma),
  providers: [Google],
  pages: {
    signIn: "/auth/signin",
  },
  events: {
    async createUser({ user }) {
      // Auto-assign admin role
      const adminEmails = (process.env.ADMIN_EMAILS || "").split(",").map(e => e.trim().toLowerCase()).filter(Boolean);
      if (user.email && adminEmails.includes(user.email.toLowerCase())) {
        await prisma.user.update({
          where: { id: user.id! },
          data: { role: "admin" },
        });
        console.log(`[AUTH] Admin role assigned to ${user.email}`);
      }

      // Auto-generate serial key on signup
      if (user.id) {
        const key = `ET-${uuidv4().slice(0, 8).toUpperCase()}-${uuidv4().slice(0, 4).toUpperCase()}`;
        await prisma.serialKey.create({
          data: {
            key,
            userId: user.id,
            plan: "free",
          },
        });
        console.log(`[AUTH] Serial key created for user ${user.email}: ${key}`);

        // TODO: Send serial key email via Resend
        // const resend = new Resend(process.env.RESEND_API_KEY);
        // await resend.emails.send({ ... });
      }
    },
  },
  callbacks: {
    async session({ session, user }) {
      if (session.user) {
        session.user.id = user.id;
        (session.user as any).role = (user as any).role || "user";
      }
      return session;
    },
  },
});
