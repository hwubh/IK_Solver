using System.Collections.Generic;

namespace UnityEngine.PathTracing.Core
{
    // The type parameter T is only used as a tag, preventing different kinds of handles from being mixed together.
    internal readonly struct Handle<T>
    {
        public readonly int Value;

        public Handle(int value)
        {
            Value = value;
        }

        public static readonly Handle<T> Invalid = new Handle<T>(-1);

        public bool IsValid() => Value >= 0;

        // Value type semantics
        public override int GetHashCode() => Value.GetHashCode();
        public override bool Equals(object obj) => obj is Handle<T> other && other.Value == Value;
        public override string ToString() => $"Handle<{typeof(T).Name}>({Value})";
        public static bool operator ==(Handle<T> a, Handle<T> b) => a.Value == b.Value;
        public static bool operator !=(Handle<T> a, Handle<T> b) => a.Value != b.Value;
    }

    // Keeps track of allocated instance handles. Reuses freed handles.
    internal class HandleSet<T>
    {
        private readonly Stack<Handle<T>> _freeHandles = new();
        private int _nextHandleIndex;

        public Handle<T> Add()
        {
            if (_freeHandles.Count > 0)
                return _freeHandles.Pop();

            return new Handle<T>(_nextHandleIndex++);
        }

        public void Remove(Handle<T> handle)
        {
            Debug.Assert(!_freeHandles.Contains(handle));
            _freeHandles.Push(handle);
        }

        public void Clear()
        {
            _freeHandles.Clear();
            _nextHandleIndex = 0;
        }
    }
}
