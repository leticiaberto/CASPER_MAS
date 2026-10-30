from collections import defaultdict
'''
Situation	                                Trust starts as	                                    source
Interacted before	                        Replay of that partner’s episodes	                direct
Never interacted, similar partner found	    Similar partner’s trust, shrunk toward neutral	    similar
Never interacted, nothing similar	        Unset (None)	                                    none
'''

class TrustModel:
    def __init__(self, contexts, learning_rate=0.2, neutral=0.5):
        self.contexts = contexts
        self.lr = learning_rate
        self.neutral = neutral                  # starting point for the first update
        self.trust_scores = defaultdict(dict)   # skill -> context -> trust
        self.source = defaultdict(dict)         # skill -> context -> "direct" | "similar"
        self.count = defaultdict(dict)          # skill -> context -> number of episodes
 
    def get(self, skill, context):
        return self.trust_scores.get(skill, {}).get(context)
 
    def get_source(self, skill, context):
        return self.source.get(skill, {}).get(context)
 
    def get_count(self, skill, context):
        return self.count.get(skill, {}).get(context, 0)
 
    def restore(self, skill, context, value, source="direct", count=0):
        """Load a stored trust value from memory as-is (no learning-rate update)."""
        if self.contexts is not None and context not in self.contexts:
            return
        self.trust_scores[skill][context] = max(0.0, min(1.0, float(value)))
        self.source[skill][context] = source      # "direct" (learned from tasks) | "prior" (self-report only)
        self.count[skill][context] = count
 
    def set_prior(self, skill, context, value):
        """Set a borrowed starting value. Never overwrites direct evidence."""
        if self.get_source(skill, context) == "direct":
            return
        self.trust_scores[skill][context] = max(0.0, min(1.0, value))
        self.source[skill][context] = "similar"
 
    def update(self, episode, clamp=True):            
        skill, context = episode["skill"], episode["context"]
        if self.contexts is not None and context not in self.contexts:
            return
 
        outcome = float(episode["outcome"])
        if clamp:
            outcome = max(0.0, min(1.0, outcome))
 
        old = self.get(skill, context)
        if old is None:
            old = self.neutral      # first evidence: start neutral, not at 0
 
        new = old + self.lr * (outcome - old)
        if clamp:
            new = max(0.0, min(1.0, new))
 
        self.trust_scores[skill][context] = new
        self.source[skill][context] = "direct"
        self.count[skill][context] = self.count[skill].get(context, 0) + 1
 
    def pairs(self):
        return {(s, c) for s, ctxs in self.trust_scores.items() for c in ctxs}
 
    def export_trust_scores(self):
        """
        Returns:
            dict[context][skill] = trust_score
        """
        result = {}
        for skill, ctxs in self.trust_scores.items():
            for context, value in ctxs.items():
                result.setdefault(context, {})[skill] = value
        return result