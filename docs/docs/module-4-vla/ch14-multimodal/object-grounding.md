# Object Grounding: Language to Vision

## Learning Objectives

- Map language descriptions to visual objects
- Use CLIP for zero-shot object recognition
- Implement spatial reasoning (left/right, on/under)
- Handle ambiguous references

**Estimated Time**: 3 hours

---

## Visual Grounding with CLIP

```python
import torch
import clip
from PIL import Image

class ObjectGrounder:
    def __init__(self):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model, self.preprocess = clip.load("ViT-B/32", device=self.device)

    def find_object(self, image, text_query):
        """
        Find object in image matching text description

        Args:
            image: PIL Image
            text_query: "red cup", "blue ball", etc.

        Returns:
            Bounding box and confidence
        """
        # Preprocess image
        image_input = self.preprocess(image).unsqueeze(0).to(self.device)

        # Encode text
        text_input = clip.tokenize([text_query]).to(self.device)

        # Compute similarity
        with torch.no_grad():
            image_features = self.model.encode_image(image_input)
            text_features = self.model.encode_text(text_input)

            # Cosine similarity
            similarity = (image_features @ text_features.T).item()

        return similarity

# Usage
grounder = ObjectGrounder()

image = Image.open("scene.jpg")
score = grounder.find_object(image, "red cup on the table")
print(f"Match confidence: {score:.3f}")
```

Continue to [Task Monitoring →](./task-monitoring.md)
