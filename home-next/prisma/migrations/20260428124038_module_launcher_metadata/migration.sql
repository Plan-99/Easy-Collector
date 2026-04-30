-- AlterTable
ALTER TABLE "Module" ADD COLUMN     "assetName" TEXT,
ADD COLUMN     "dependencies" TEXT[] DEFAULT ARRAY[]::TEXT[],
ADD COLUMN     "installByDefault" BOOLEAN NOT NULL DEFAULT false,
ADD COLUMN     "required" BOOLEAN NOT NULL DEFAULT false;
