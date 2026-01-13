package frc.robot.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.BiConsumer;
import java.util.stream.Stream;

/**
 * Utility methods for working with Java Streams and iterables.
 *
 * This class provides functional utilities for stream operations that aren't
 * natively available in the Stream API, such as zipping, enumerating, and
 * unit conversion for WPILib measurements.
 */
public class IterUtil {

    /** Private constructor to prevent instantiation. */
    private IterUtil() {}

    /**
     * Zips two streams together, applying a consumer function to each pair of elements.
     *
     * Iterates through both streams in parallel, calling the consumer with each pair
     * of elements. If the streams have different lengths, iteration stops when the
     * shorter stream is exhausted.
     *
     * Example: Pairing module states with module names for dashboard display
     *
     * @param <T> The element type of the first stream
     * @param <K> The element type of the second stream
     * @param a The first stream
     * @param b The second stream
     * @param consumer Function to apply to each pair of elements
     */
    public static <T, K> void zipThen(
        Stream<T> a,
        Stream<K> b,
        BiConsumer<T, K> consumer
    ) {
        Iterator<T> aIterator = a.iterator();
        Iterator<K> bIterator = b.iterator();
        while (aIterator.hasNext() && bIterator.hasNext()) {
            consumer.accept(aIterator.next(), bIterator.next());
        }
    }

    /**
     * Zips two streams together, returning a stream of entry pairs.
     *
     * Combines elements from two streams into a stream of Map.Entry pairs.
     * If the streams have different lengths, the longer stream is truncated.
     *
     * @param <T> The element type of the first stream
     * @param <K> The element type of the second stream
     * @param a The first stream
     * @param b The second stream
     * @return A stream of Map.Entry pairs, one for each pair of elements
     */
    public static <T, K> Stream<Entry<T, K>> zip(Stream<T> a, Stream<K> b) {
        Iterator<T> aIterator = a.iterator();
        Iterator<K> bIterator = b.iterator();
        return Stream.iterate(
            true,
            i -> aIterator.hasNext() && bIterator.hasNext(),
            i -> i
        ).map(i -> Map.entry(aIterator.next(), bIterator.next()));
    }

    /**
     * Adds indices to stream elements, creating pairs of (index, element).
     *
     * Useful for iterating over a stream when you need to know the position
     * of each element. The index starts at 0.
     *
     * @param <T> The element type of the stream
     * @param a The stream to enumerate
     * @return A stream of Map.Entry pairs where the key is the index (0-based) and value is the element
     */
    public static <T> Stream<Entry<Integer, T>> enumerate(Stream<T> a) {
        Iterator<T> aIterator = a.iterator();
        return Stream.iterate(0, i -> aIterator.hasNext(), i -> i + 1).map(i ->
            Map.entry(i, aIterator.next())
        );
    }

    /**
     * Converts a stream of WPILib measurements to an array of double values.
     *
     * This is useful for converting measurement streams (Distance, Angle, etc.)
     * to raw double values for array operations.
     *
     * @param <U> The unit type (e.g., Meter, Radian)
     * @param <M> The measurement type (e.g., Distance, Angle)
     * @param a The stream of measurements to convert
     * @param unit The unit to convert the measurements to
     * @return An array of double values in the specified unit
     */
    public static <U extends Unit, M extends Measure<U>> double[] deunit(
        Stream<M> a,
        U unit
    ) {
        return a.mapToDouble(m -> m.in(unit)).toArray();
    }

    /**
     * Enumerate and apply a consumer to each indexed element in a stream.
     *
     * Similar to a for-each loop with index, this applies a function to each
     * element along with its 0-based index.
     *
     * @param <T> The element type of the stream
     * @param a The stream to enumerate
     * @param consumer Function to apply to each (index, element) pair
     */
    public static <T> void enumerateThen(
        Stream<T> a,
        BiConsumer<Integer, T> consumer
    ) {
        Iterator<T> aIterator = a.iterator();
        int i = 0;
        while (aIterator.hasNext()) {
            consumer.accept(i, aIterator.next());
            i++;
        }
    }
}
