#pragma once
// Reaches the HAL handles that Arduino_AdvancedAnalog keeps private, so the
// sketch can report and clear a stuck ADC/DMA state (see adcSanitize() in the .ino).
//
// AdvancedADC holds `adc_descr_t *descr` (private). The template below is the
// standard explicit-instantiation route to a private member pointer. The struct
// is a copy of adc_descr_t from AdvancedADC.cpp (library 1.5.0). The sketch checks
// that adc.Instance and dma.Instance hold the expected peripherals before it writes
// through this mirror, so a changed layout in a future library version is refused,
// not written through. Kept in a header so the IDE's prototype generator doesn't see it.
#include <Arduino_AdvancedAnalog.h>

namespace adc_access {
template <typename Tag, typename Tag::type M>
struct Grab { friend typename Tag::type get(Tag) { return M; } };
struct DescrTag { typedef adc_descr_t* AdvancedADC::*type; friend type get(DescrTag); };
template struct Grab<DescrTag, &AdvancedADC::descr>;
}

struct AdcDescrMirror {            // == adc_descr_t in AdvancedADC.cpp 1.5.0
    ADC_HandleTypeDef adc;
    DMA_HandleTypeDef dma;
    IRQn_Type         dma_irqn;
    TIM_HandleTypeDef tim;
    uint32_t          tim_trig;
    void*             pool;
    void*             dmabuf[2];
};

static inline AdcDescrMirror* adcDescr(AdvancedADC& a) {
    return reinterpret_cast<AdcDescrMirror*>(a.*get(adc_access::DescrTag()));
}
// The layout is trusted only if the handles point at the peripherals the library
// assigns: ADC1 -> DMA1_Stream1, ADC2 -> DMA1_Stream2, ADC3 -> DMA1_Stream3.
static inline bool adcDescrValid(const AdcDescrMirror* d) {
    if (!d) return false;
    void* dma = d->dma.Instance;
    return (d->adc.Instance == ADC1 && dma == DMA1_Stream1) ||
           (d->adc.Instance == ADC2 && dma == DMA1_Stream2) ||
           (d->adc.Instance == ADC3 && dma == DMA1_Stream3);
}
