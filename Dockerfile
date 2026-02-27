# Use official Node.js LTS image
FROM node:18-alpine

# Install required system dependencies
RUN apk add --no-cache \
    bash \
    openssh-client \
    sshpass \
    python3 \
    py3-pip \
    build-base \
    python3-dev \
    coreutils

# Set working directory
WORKDIR /app

# Copy package files
COPY package*.json ./

# Install Node.js dependencies
RUN npm install --production

# Copy application files
COPY . .

# Expose port 8009
EXPOSE 8009

# Set environment variables
ENV NODE_ENV=production
ENV PORT=8009

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD node -e "require('http').get('http://localhost:8009', (r) => {process.exit(r.statusCode === 200 ? 0 : 1)})"

# Start the application
CMD ["npm", "start"]
