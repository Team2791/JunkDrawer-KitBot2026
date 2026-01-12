package frc.robot.util;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.BiConsumer;
import java.util.stream.Stream;

public class IterUtil {

    private IterUtil() {}

    /**
     * Zips two streams together, applying a consumer to each pair of elements.
     * If one stream is longer than the other, the consumer will not be applied to the remaining elements.
     *
     * @param <T>      The type of the first stream
     * @param <K>      The type of the second stream
     * @param a        The first stream
     * @param b        The second stream
     * @param consumer The consumer to apply to each pair of elements
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
     * Zips two streams together, returning a stream of pairs of elements.
     * If one stream is longer than the other, the remaining elements will be ignored.
     *
     * @param <T> The type of the first stream
     * @param <K> The type of the second stream
     * @param a   The first stream
     * @param b   The second stream
     * @return A stream of pairs of elements
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
     * Adds indices to a stream of elements.
     *
     * @param <T> The type of the stream
     * @param a   The stream
     * @return A stream of pairs of indices and elements
     */
    public static <T> Stream<Entry<Integer, T>> enumerate(Stream<T> a) {
        Iterator<T> aIterator = a.iterator();
        return Stream.iterate(0, i -> aIterator.hasNext(), i -> i + 1).map(i ->
            Map.entry(i, aIterator.next())
        );
    }

    /**
     * Convert uom stream to a double array
     *
     * @param <U> The unit, e.g. Meter
     * @param <M> The type of the stream, e.g. Distance
     * @param a   The stream
     * @return A stream of doubles
     */
    public static <U extends Unit, M extends Measure<U>> double[] deunit(
        Stream<M> a,
        U unit
    ) {
        return a.mapToDouble(m -> m.in(unit)).toArray();
    }

    /**
     * Adds indices to a stream of elements, applying a consumer to each pair of indices and elements.
     *
     * @param <T>      The type of the stream
     * @param a        The stream
     * @param consumer The consumer to apply to each pair of indices and elements
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
