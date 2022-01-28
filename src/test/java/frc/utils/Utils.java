package frc.utils;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.spy;

import java.lang.reflect.Field;

public class Utils {
    public static Object GetPrivate(Object base, String fieldName) {
        Field field;
        try {
            field = base.getClass().getDeclaredField(fieldName);
        } catch (NoSuchFieldException | SecurityException e) {
            e.printStackTrace();
            return null;
        }
        field.setAccessible(true);
        try {
            return field.get(base);
        } catch (IllegalArgumentException | IllegalAccessException e) {
            e.printStackTrace();
            return null;
        }
    }
    public static Object ReflectAndSpy(Object base, String fieldName) {
        Field field;
        Object mock;

        try {
            field = base.getClass().getDeclaredField(fieldName);
        } catch (NoSuchFieldException | SecurityException e) {
            e.printStackTrace();
            return null;
        }

        field.setAccessible(true);
        
        try {
            mock = spy(field.get(base));
        } catch (IllegalArgumentException | IllegalAccessException e1) {
            e1.printStackTrace();
            return null;
        }

        try {
            field.set(base, mock);
        } catch (IllegalArgumentException | IllegalAccessException e) {
            e.printStackTrace();
            return null;
        }

        return mock;
    }
    public static Object ReflectAndMock(Object base, String fieldName) {
        Field field;
        Object mock;

        try {
            field = base.getClass().getDeclaredField(fieldName);
        } catch (NoSuchFieldException | SecurityException e) {
            e.printStackTrace();
            return null;
        }

        field.setAccessible(true);
        
        mock = mock(field.getType());

        try {
            field.set(base, mock);
        } catch (IllegalArgumentException | IllegalAccessException e) {
            e.printStackTrace();
            return null;
        }

        return mock;
    }
}