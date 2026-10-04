#include "graphics_device.hpp"
#include <QQuickWindow>
#include <QQuickGraphicsDevice>
#include <QDebug>
#ifdef Q_OS_WIN
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <dxgi1_6.h>
#include <wrl/client.h>
#endif

void configureGraphicsDevice(QQuickWindow& window) {
#ifdef Q_OS_WIN
    const auto backend=qgetenv("QSG_RHI_BACKEND");
    if (qEnvironmentVariableIntValue("FD_DEFAULT_GPU")!=0 ||
        qEnvironmentVariableIsSet("QT_D3D_ADAPTER_INDEX") ||
        qEnvironmentVariableIsSet("QT_D3D_ADAPTER_LUID") ||
        qEnvironmentVariableIsSet("QSG_RHI_PREFER_SOFTWARE_RENDERER") ||
        qgetenv("QT_QUICK_BACKEND")=="software" ||
        (!backend.isEmpty() && backend!="d3d11" && backend!="d3d12")) return;
    using Microsoft::WRL::ComPtr;
    ComPtr<IDXGIFactory6> factory;
    if (FAILED(CreateDXGIFactory1(IID_PPV_ARGS(&factory)))) return;
    ComPtr<IDXGIAdapter1> adapter;
    if (FAILED(factory->EnumAdapterByGpuPreference(0,DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE,IID_PPV_ARGS(&adapter)))) return;
    DXGI_ADAPTER_DESC1 description{};
    if (FAILED(adapter->GetDesc1(&description)) || (description.Flags & DXGI_ADAPTER_FLAG_SOFTWARE)) return;
    window.setGraphicsDevice(QQuickGraphicsDevice::fromAdapter(description.AdapterLuid.LowPart,description.AdapterLuid.HighPart));
    window.setProperty("preferredGraphicsAdapter",QString::fromWCharArray(description.Description));
    qInfo().noquote() << "Graphics adapter:" << QString::fromWCharArray(description.Description);
#else
    Q_UNUSED(window);
#endif
}
