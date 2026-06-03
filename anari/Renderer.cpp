// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#include "Renderer.h"
#include <cstring>

namespace barney_device {

Renderer::Renderer(BarneyGlobalState *s)
    : Object(ANARI_RENDERER, s), m_backgroundImage(this)
{
  barneyRenderer = bnRendererCreate(deviceState()->tether->context, "default");
}

Renderer::~Renderer()
{
  bnRelease(barneyRenderer);
}

void Renderer::commitParameters()
{
  m_pixelSamples = getParam<int>("pixelSamples", 1);
  m_ambientRadiance = getParam<float>("ambientRadiance", 1.f);
  m_crosshairs = getParam<bool>("crosshairs", false);
  m_denoise = getParam<bool>("denoise", true);
  m_fadeOutDenoiser = getParam<bool>("fadeOutDenoiser", true);
  m_upscale = getParam<bool>("upscale", false);
  m_background = getParam<math::float4>("background", math::float4(0, 0, 0, 1));
  m_backgroundImage = getParamObject<Array2D>("background");
  m_cutPlane = getParam<math::float4>("cutPlane", math::float4(0, 0, 0, -1e30f));
}

bool Renderer::getProperty(const std::string_view &name,
    ANARIDataType type,
    void *ptr,
    uint64_t size,
    uint32_t flags)
{
  (void)size;
  (void)flags;
  if (name == "denoise" && type == ANARI_BOOL) {
    helium::writeToVoidP(ptr, m_denoise);
    return true;
  }
  if (name == "pixelSamples" && type == ANARI_INT32) {
    helium::writeToVoidP(ptr, m_pixelSamples);
    return true;
  }
  if (name == "crosshairs" && type == ANARI_BOOL) {
    helium::writeToVoidP(ptr, m_crosshairs);
    return true;
  }
  if (name == "ambientRadiance" && type == ANARI_FLOAT32) {
    helium::writeToVoidP(ptr, m_ambientRadiance);
    return true;
  }
  if (name == "background" && type == ANARI_FLOAT32_VEC4) {
    std::memcpy(ptr, &m_background, sizeof(m_background));
    return true;
  }
  if (name == "cutPlane" && type == ANARI_FLOAT32_VEC4) {
    std::memcpy(ptr, &m_cutPlane, sizeof(m_cutPlane));
    return true;
  }
  return Object::getProperty(name, type, ptr, size, flags);
}

void Renderer::finalize()
{
  bnSetVec(barneyRenderer, "bgColor", m_background);
  bnSet1i(barneyRenderer, "crosshairs", (int)m_crosshairs);
  bnSet1i(barneyRenderer, "pathsPerPixel", (int)m_pixelSamples);
  bnSet1f(barneyRenderer, "ambientRadiance", m_ambientRadiance);
  bnSet4f(barneyRenderer, "cutPlane",
          m_cutPlane.x, m_cutPlane.y, m_cutPlane.z, m_cutPlane.w);

  if (m_backgroundImage) {
    int sx = m_backgroundImage->size().x;
    int sy = m_backgroundImage->size().y;
    const bn_float4 *texels
      = (const bn_float4 *)m_backgroundImage->data();
    barneyBackgroundImage
      = bnTexture2DCreate(deviceState()->tether->context,-1,
                          BN_FLOAT4,sx,sy,
                          texels,
                          BN_TEXTURE_LINEAR,
                          BN_TEXTURE_CLAMP,BN_TEXTURE_CLAMP);
    bnSetObject(barneyRenderer,"bgTexture",barneyBackgroundImage);
  } else {
    if (barneyBackgroundImage) {
      bnRelease(barneyBackgroundImage);
      barneyBackgroundImage = 0;
      bnSetObject(barneyRenderer,"bgTexture",0);
    }
  }
  bnCommit(barneyRenderer);
}

bool Renderer::crosshairs() const
{
  return m_crosshairs;
}

bool Renderer::denoise() const
{
  return m_denoise;
}

bool Renderer::fadeOutDenoiser() const
{
  return m_fadeOutDenoiser;
}

bool Renderer::upscale() const
{
  return m_upscale;
}

bool Renderer::isValid() const
{
  return barneyRenderer != 0;
}

} // namespace barney_device

BARNEY_ANARI_TYPEFOR_DEFINITION(barney_device::Renderer *);
