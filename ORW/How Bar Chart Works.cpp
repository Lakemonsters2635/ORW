// PlotBars:

template <typename T>
void PlotBars(const char* label_id, const T* values, int count, double width = 0.67, double shift = 0, int offset = 0, int stride = sizeof(T)) {
    GetterBarV<T> getter(values, shift, count, offset, stride);
    PlotBarsEx(label_id, getter, width);
}

template <typename T>
struct GetterBarV {
    const T* Ys; double XShift; int Count; int Offset; int Stride;
    GetterBarV(const T* ys, double xshift, int count, int offset, int stride) { Ys = ys; XShift = xshift; Count = count; Offset = offset; Stride = stride; }
    // Simplest case: return a plot point x=idx, y=data[idx]
    inline ImPlotPoint operator()(int idx) const { return ImPlotPoint((double)idx + (double)XShift, (double)OffsetAndStride(Ys, idx, Count, Offset, Stride)); }
};


// Offsets and strides a data buffer.  In simple case, just return datum at idx

template <typename T>
inline T OffsetAndStride(const T* data, int idx, int count, int offset, int stride) {
    idx = ImPosMod(offset + idx, count);
    return *(const T*)(const void*)((const unsigned char*)data + (size_t)idx * stride);
}


template <typename Getter>
void PlotBarsEx(const char* label_id, const Getter& getter, double width) {
    if (BeginItem(label_id, ImPlotCol_Fill)) {
        const double half_width = width / 2;
        if (FitThisFrame()) {
            for (int i = 0; i < getter.Count; ++i) {
                ImPlotPoint p = getter(i);
                FitPoint(ImPlotPoint(p.x - half_width, p.y));
                FitPoint(ImPlotPoint(p.x + half_width, 0));
            }
        }
        const ImPlotNextItemData& s = GetItemData();
        ImDrawList& DrawList = *GetPlotDrawList();
        ImU32 col_line = ImGui::GetColorU32(s.Colors[ImPlotCol_Line]);
        ImU32 col_fill = ImGui::GetColorU32(s.Colors[ImPlotCol_Fill]);
        bool  rend_line = s.RenderLine;
        if (s.RenderFill && col_line == col_fill)
            rend_line = false;
        for (int i = 0; i < getter.Count; ++i) {
            ImPlotPoint p = getter(i);
            if (p.y == 0)
                continue;
            ImVec2 a = PlotToPixels(p.x - half_width, p.y);
            ImVec2 b = PlotToPixels(p.x + half_width, 0);
            if (s.RenderFill)
                DrawList.AddRectFilled(a, b, col_fill);
            if (rend_line)
                DrawList.AddRect(a, b, col_line, 0, ImDrawCornerFlags_All, s.LineWeight);
        }
        EndItem();
    }
}
