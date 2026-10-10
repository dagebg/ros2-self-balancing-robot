# syntax=docker/dockerfile:1
# Web app image: the static build served by nginx. The Node stage runs on the build machine's
# architecture, so a multi-arch image needs no emulated npm install.

FROM --platform=$BUILDPLATFORM node:24-alpine AS build
WORKDIR /app
COPY webapp/package.json webapp/package-lock.json ./
RUN npm ci
COPY webapp/ ./
RUN npm run build

FROM nginx:1.28-alpine
COPY docker/nginx.conf /etc/nginx/conf.d/default.conf
COPY --from=build /app/dist /usr/share/nginx/html
