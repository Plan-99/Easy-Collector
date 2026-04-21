FROM node:20-slim

WORKDIR /app

# Install Quasar CLI globally
RUN npm install -g @quasar/cli

# Bundle source into image (overridden by volume mount in dev)
COPY frontend/ ./

# Install dependencies (needs full project for quasar prepare hook)
RUN npm ci --legacy-peer-deps || npm install --legacy-peer-deps

EXPOSE 5173

COPY frontend/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
