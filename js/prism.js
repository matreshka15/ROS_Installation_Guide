// Prism.js - Lightweight syntax highlighter
// Minimal version for Bash syntax highlighting

var Prism = {
    languages: {
        bash: {
            'comment': /#.*/,
            'string': {
                pattern: /(['"])(?:\\.|(?!\1)[^\\\r\n])*\1/,
                greedy: true
            },
            'keyword': /\b(?:if|then|else|elif|fi|for|in|do|done|while|break|continue|case|esac|function|return|exit|set|unset|declare|export|readonly|alias|unalias|source|.\s|eval|exec|trap|umask|builtin|type|command|hash|help|logout|history)\b/,
            'function': /\w+(?=\s*\()/,
            'variable': /\$\w+|\$\{[^}]+\}/,
            'operator': /\+|-|\*|\/|%|=|==|!=|<=|>=|<|>|!|&&|\|\|/,
            'number': /\b\d+(?:\.\d+)?\b/,
            'punctuation': /[;:,\[\](){}]/
        }
    },
    highlight: function (code, language) {
        var grammar = Prism.languages[language];
        if (!grammar) {
            return code;
        }
        
        var tokens = [];
        var token;
        var index = 0;
        var match;
        var pattern;
        var type;
        
        // Simple regex-based highlighting
        var codeWithHighlighting = code;
        
        // Highlight comments
        codeWithHighlighting = codeWithHighlighting.replace(/(#.*)/g, '<span class="token comment">$1</span>');
        
        // Highlight strings
        codeWithHighlighting = codeWithHighlighting.replace(/(['"])(?:\\.|(?!\1)[^\\\r\n])*\1/g, '<span class="token string">$&</span>');
        
        // Highlight variables
        codeWithHighlighting = codeWithHighlighting.replace(/(\$\w+|\$\{[^}]+\})/g, '<span class="token variable">$1</span>');
        
        // Highlight keywords
        var keywords = /\b(if|then|else|elif|fi|for|in|do|done|while|break|continue|case|esac|function|return|exit|set|unset|declare|export|readonly|alias|unalias|source|\.\s|eval|exec|trap|umask|builtin|type|command|hash|help|logout|history)\b/g;
        codeWithHighlighting = codeWithHighlighting.replace(keywords, '<span class="token keyword">$1</span>');
        
        // Highlight operators
        codeWithHighlighting = codeWithHighlighting.replace(/(\+|-|\*|\/|%|=|==|!=|<=|>=|<|>|!|&&|\|\|)/g, '<span class="token operator">$1</span>');
        
        // Highlight numbers
        codeWithHighlighting = codeWithHighlighting.replace(/\b\d+(?:\.\d+)?\b/g, '<span class="token number">$&</span>');
        
        return codeWithHighlighting;
    }
};

// Prism CSS for basic styling
var prismCSS = `
/* Prism.js Basic Theme */
.token.comment,
.token.prolog,
.token.doctype,
.token.cdata {
    color: #6a9955;
    font-style: italic;
}

.token.punctuation {
    color: #d4d4d4;
}

.token.property,
.token.tag,
.token.boolean,
.token.number,
.token.constant,
.token.symbol,
.token.deleted {
    color: #b5cea8;
}

.token.selector,
.token.attr-name,
.token.string,
.token.char,
.token.builtin,
.token.inserted {
    color: #ce9178;
}

.token.operator,
.token.entity,
.token.url,
.language-css .token.string,
.style .token.string {
    color: #d4d4d4;
}

.token.atrule,
.token.attr-value,
.token.keyword {
    color: #569cd6;
}

.token.function {
    color: #dcdcaa;
}

.token.regex,
.token.important,
.token.variable {
    color: #d16969;
}

.token.important,
.token.bold {
    font-weight: bold;
}

.token.italic {
    font-style: italic;
}

.token.entity {
    cursor: help;
}
`;

// Add CSS to the document
var styleElement = document.createElement('style');
styleElement.textContent = prismCSS;
document.head.appendChild(styleElement);

// Initialize highlighting when DOM is loaded
document.addEventListener('DOMContentLoaded', function() {
    var codeBlocks = document.querySelectorAll('pre code');
    codeBlocks.forEach(function(block) {
        var language = block.getAttribute('class') || 'bash';
        if (language.startsWith('language-')) {
            language = language.slice(9);
        }
        block.innerHTML = Prism.highlight(block.textContent, language);
    });
});