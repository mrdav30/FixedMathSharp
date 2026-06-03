using BenchmarkDotNet.Attributes;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;

namespace FixedMathSharp.Benchmarks;

internal sealed class BenchmarkCatalog
{
    private static readonly string[] _benchmarkSuffixes = new[] { "Benchmarks", "Benchmark" };
    private static readonly HashSet<string> _selectionQualifiers = new(StringComparer.OrdinalIgnoreCase) { };
    private static readonly Dictionary<string, string[]> _aliasSynonyms = new(StringComparer.OrdinalIgnoreCase) { };

    private readonly Dictionary<string, Type[]> _aliasLookup;
    private readonly KeyValuePair<string, Type[]>[] _displayAliases;

    private BenchmarkCatalog(Dictionary<string, Type[]> aliasLookup, KeyValuePair<string, Type[]>[] displayAliases)
    {
        _aliasLookup = aliasLookup;
        _displayAliases = displayAliases;
    }

    public static BenchmarkCatalog Create(Assembly assembly)
    {
        Type[] benchmarkTypes = assembly
            .GetTypes()
            .Where(type => type.IsClass && !type.IsAbstract && ContainsBenchmarkMethods(type))
            .OrderBy(type => type.Name, StringComparer.Ordinal)
            .ToArray();

        var aliasLookup = new Dictionary<string, HashSet<Type>>(StringComparer.OrdinalIgnoreCase);
        var displayAliases = new Dictionary<string, HashSet<Type>>(StringComparer.OrdinalIgnoreCase);

        foreach (Type benchmarkType in benchmarkTypes)
        {
            string strippedName = StripBenchmarkSuffix(benchmarkType.Name);
            string[] words = SplitWords(strippedName);
            string specificAlias = string.Join("-", words.Select(word => word.ToLowerInvariant()));
            string selectionAlias = GetSelectionAlias(words);

            AddHiddenAlias(aliasLookup, benchmarkType.Name, benchmarkType);
            AddHiddenAlias(aliasLookup, strippedName, benchmarkType);
            AddDisplayAlias(aliasLookup, displayAliases, specificAlias, benchmarkType);

            if (!string.Equals(selectionAlias, specificAlias, StringComparison.OrdinalIgnoreCase))
                AddDisplayAlias(aliasLookup, displayAliases, selectionAlias, benchmarkType);

            if (_aliasSynonyms.TryGetValue(selectionAlias, out string[] selectionSynonyms))
            {
                foreach (string synonym in selectionSynonyms)
                    AddDisplayAlias(aliasLookup, displayAliases, synonym, benchmarkType);
            }

            if (_aliasSynonyms.TryGetValue(specificAlias, out string[] specificSynonyms))
            {
                foreach (string synonym in specificSynonyms)
                    AddDisplayAlias(aliasLookup, displayAliases, synonym, benchmarkType);
            }
        }

        return new BenchmarkCatalog(
            aliasLookup.ToDictionary(
                entry => entry.Key,
                entry => entry.Value.OrderBy(type => type.Name, StringComparer.Ordinal).ToArray(),
                StringComparer.OrdinalIgnoreCase),
            displayAliases
                .OrderBy(entry => entry.Key, StringComparer.OrdinalIgnoreCase)
                .Select(entry => new KeyValuePair<string, Type[]>(
                    entry.Key,
                    entry.Value.OrderBy(type => type.Name, StringComparer.Ordinal).ToArray()))
                .ToArray());
    }

    public Type[] Resolve(string[] aliases, out string unknownAlias)
    {
        var selectedTypes = new HashSet<Type>();

        foreach (string alias in aliases)
        {
            string normalizedAlias = NormalizeAlias(alias);
            if (!_aliasLookup.TryGetValue(normalizedAlias, out Type[] matchedTypes))
            {
                unknownAlias = alias;
                return Array.Empty<Type>();
            }

            foreach (Type matchedType in matchedTypes)
                selectedTypes.Add(matchedType);
        }

        unknownAlias = null;
        return selectedTypes
            .OrderBy(type => type.Name, StringComparer.Ordinal)
            .ToArray();
    }

    public void WriteAvailableSelections(TextWriter writer)
    {
        writer.WriteLine("Available benchmark selections:");
        foreach (KeyValuePair<string, Type[]> alias in _displayAliases)
        {
            writer.Write("  ");
            writer.Write(alias.Key.PadRight(24));
            writer.Write(" -> ");
            writer.WriteLine(string.Join(", ", alias.Value.Select(type => type.Name)));
        }
    }

    private static void AddDisplayAlias(
        Dictionary<string, HashSet<Type>> aliasLookup,
        Dictionary<string, HashSet<Type>> displayAliases,
        string alias,
        Type benchmarkType)
    {
        AddLookupAlias(aliasLookup, alias, benchmarkType);
        AddDisplayEntry(displayAliases, alias, benchmarkType);
    }

    private static void AddHiddenAlias(Dictionary<string, HashSet<Type>> aliasLookup, string alias, Type benchmarkType)
    {
        AddLookupAlias(aliasLookup, alias, benchmarkType);
    }

    private static void AddLookupAlias(Dictionary<string, HashSet<Type>> aliases, string alias, Type benchmarkType)
    {
        string normalizedAlias = NormalizeAlias(alias);
        if (!aliases.TryGetValue(normalizedAlias, out HashSet<Type> benchmarkTypes))
        {
            benchmarkTypes = new HashSet<Type>();
            aliases.Add(normalizedAlias, benchmarkTypes);
        }

        benchmarkTypes.Add(benchmarkType);
    }

    private static void AddDisplayEntry(Dictionary<string, HashSet<Type>> aliases, string alias, Type benchmarkType)
    {
        if (!aliases.TryGetValue(alias, out HashSet<Type> benchmarkTypes))
        {
            benchmarkTypes = new HashSet<Type>();
            aliases.Add(alias, benchmarkTypes);
        }

        benchmarkTypes.Add(benchmarkType);
    }

    private static bool ContainsBenchmarkMethods(Type type)
    {
        return type
            .GetMethods(BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic)
            .Any(method => method.GetCustomAttributes(typeof(BenchmarkAttribute), false).Length > 0);
    }

    private static string StripBenchmarkSuffix(string typeName)
    {
        foreach (string suffix in _benchmarkSuffixes)
        {
            if (typeName.EndsWith(suffix, StringComparison.Ordinal))
                return typeName.Substring(0, typeName.Length - suffix.Length);
        }

        return typeName;
    }

    private static string GetSelectionAlias(string[] words)
    {
        if (words.Length > 1 && _selectionQualifiers.Contains(words[words.Length - 1]))
            return string.Join("-", words.Take(words.Length - 1).Select(word => word.ToLowerInvariant()));

        return string.Join("-", words.Select(word => word.ToLowerInvariant()));
    }

    private static string[] SplitWords(string value)
    {
        var words = new List<string>();
        var currentWord = new StringBuilder();

        for (int i = 0; i < value.Length; i++)
        {
            char currentCharacter = value[i];
            if (currentWord.Length > 0 && char.IsUpper(currentCharacter) && ShouldSplitWord(value, i))
            {
                words.Add(currentWord.ToString());
                currentWord.Clear();
            }

            currentWord.Append(currentCharacter);
        }

        if (currentWord.Length > 0)
            words.Add(currentWord.ToString());

        return words.ToArray();
    }

    private static bool ShouldSplitWord(string value, int index)
    {
        char previousCharacter = value[index - 1];
        if (char.IsLower(previousCharacter) || char.IsDigit(previousCharacter))
            return true;

        return index + 1 < value.Length && char.IsLower(value[index + 1]);
    }

    private static string NormalizeAlias(string alias)
    {
        var normalizedAlias = new StringBuilder(alias.Length);
        foreach (char character in alias)
        {
            if (char.IsLetterOrDigit(character))
                normalizedAlias.Append(char.ToLowerInvariant(character));
        }

        return normalizedAlias.ToString();
    }
}
