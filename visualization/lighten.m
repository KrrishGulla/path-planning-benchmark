function c = lighten(c, a)
%LIGHTEN  Blend colour c towards white by fraction a (0..1).
c = c + (1 - c) * a;
end
