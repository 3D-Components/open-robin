# -- Stage 1: build the React app ----------------------------------------
FROM node:22-alpine AS build

WORKDIR /app
COPY package.json package-lock.json ./
RUN npm ci

# Pass the backend URL at build time so Vite can inline it
ARG VITE_ROBIN_API_URL=http://localhost:8000
ENV VITE_ROBIN_API_URL=$VITE_ROBIN_API_URL

COPY . .
RUN npm run build

# -- Stage 2: serve with nginx -------------------------------------------
FROM nginx:alpine

# Remove default nginx page
RUN rm -rf /usr/share/nginx/html/*

# Copy the React build output
COPY --from=build /app/dist /usr/share/nginx/html

# SPA fallback: route all paths to index.html
RUN printf 'server {\n\
    listen 80;\n\
    server_name _;\n\
    root /usr/share/nginx/html;\n\
    index index.html;\n\
\n\
    location / {\n\
        try_files $uri $uri/ /index.html;\n\
    }\n\
\n\
    # Cache static assets aggressively\n\
    location ~* \\.(js|css|png|jpg|jpeg|gif|ico|svg|woff2?)$ {\n\
        expires 30d;\n\
        add_header Cache-Control "public, immutable";\n\
    }\n\
}\n' > /etc/nginx/conf.d/default.conf

EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]
