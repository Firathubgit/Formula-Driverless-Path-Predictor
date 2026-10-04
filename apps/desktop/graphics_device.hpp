#pragma once
class QQuickWindow;
// Prefer the high-performance hardware adapter on Windows; preserve explicit backend/adapter overrides.
void configureGraphicsDevice(QQuickWindow& window);
