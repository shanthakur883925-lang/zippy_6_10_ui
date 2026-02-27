#!/bin/bash

# Robot Control Web UI - Startup Script
# This script starts the web server and displays access URLs

echo "🤖 Starting Robot Control Web UI..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Install dependencies if needed
if [ ! -d "node_modules" ]; then
    echo "📦 Installing dependencies..."
    npm install
fi

# Get local IP address
LOCAL_IP=$(hostname -I | awk '{print $1}')

echo ""
echo "✅ Server will start on port 8009"
echo ""
echo "📍 Access your Robot UI at:"
echo "   • Local:    http://localhost:8009"
echo "   • Network:  http://$LOCAL_IP:8009"
echo "   • Any IP:   http://0.0.0.0:8009"
echo ""
echo "🌐 Share this link with others on your network:"
echo "   👉 http://$LOCAL_IP:8009"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Press Ctrl+C to stop the server"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Start the server
npm start
