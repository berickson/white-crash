// clamps x to be between low and high
template<class T> T clamp(T x, T low, T high) {
    if (x<low) return low;
    if (x>high) return high;
    return x;
}